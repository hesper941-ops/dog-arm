#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from collections import deque

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float32, String

from roarm_msgs.srv import MoveLineCmd


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def mean(values):
    if not values:
        return 0.0
    return float(sum(values)) / float(len(values))


def stddev(values):
    if len(values) <= 1:
        return 0.0
    avg = mean(values)
    var = sum((float(v) - avg) ** 2 for v in values) / float(len(values))
    return math.sqrt(var)


class VisualMoveItGraspNode(Node):
    def __init__(self, config_path):
        super().__init__("visual_moveit_grasp")
        self.config_path = config_path
        self.cfg = load_yaml(config_path)

        self.target_buffer = deque(maxlen=max(3, int(self.cfg["stable_target_min_frames"])))
        self.latest_target = None
        self.current_snapshot = None
        self.state = "IDLE"
        self.state_enter_time = time.time()
        self.state_action_done = False
        self.base_adjust_sent_at = 0.0
        self.done_logged = False
        self.motion_states = {
            "OPEN_GRIPPER",
            "MOVE_TO_PRE_GRASP",
            "MOVE_LINE_TO_GRASP",
            "CLOSE_GRIPPER",
            "LIFT",
        }

        self.sub_target = self.create_subscription(String, "/red_block/target_base", self.on_target_msg, 10)
        self.pub_gripper = self.create_publisher(Float32, "/gripper_cmd", 10)
        self.pub_base_adjust = self.create_publisher(String, self.cfg["base_adjust_topic"], 10)
        self.move_line_client = self.create_client(MoveLineCmd, "/move_line_cmd")
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(f"加载抓取配置: {self.config_path}")
        self.get_logger().info(
            "桥接启动: 订阅 /red_block/target_base, 调用 /move_line_cmd, 发布 /gripper_cmd"
        )

    def bool_cfg(self, key):
        value = self.cfg.get(key, False)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def set_state(self, new_state, reason=""):
        if self.state != new_state:
            text = f"状态切换: {self.state} -> {new_state}"
            if reason:
                text += f" | {reason}"
            self.get_logger().info(text)
        self.state = new_state
        self.state_enter_time = time.time()
        self.state_action_done = False

    def on_target_msg(self, msg):
        # 当前 /red_block/target_base 为 std_msgs/String，内容是 JSON。
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"目标 JSON 解析失败: {exc}")
            return

        self.latest_target = data
        if self.state not in ("IDLE", "WAIT_TARGET"):
            return

        target = self.extract_target(data)
        if target is None:
            return

        self.target_buffer.append(target)

    def extract_target(self, data):
        if not isinstance(data, dict):
            return None
        if not bool(data.get("valid", False)):
            return None
        if bool(data.get("is_hold", False)):
            # 只用实时视觉结果建 snapshot，不用上游 hold 结果盲抓。
            return None

        base = data.get("base_mm")
        pixel = data.get("pixel")
        if not isinstance(base, dict) or not isinstance(pixel, dict):
            return None

        now = time.time()
        stamp = float(data.get("stamp", now))
        age_s = max(0.0, now - stamp)
        if age_s > float(self.cfg["stage_target_max_age_s"]):
            self.get_logger().warn(f"忽略过期目标: age={age_s:.2f}s")
            return None

        try:
            return {
                "x_mm": float(base["x"]),
                "y_mm": float(base["y"]),
                "z_mm": float(base["z"]),
                "pixel_u": int(pixel["x"]),
                "pixel_v": int(pixel["y"]),
                "confidence": float(data.get("confidence", data.get("score", 0.0))),
                "stamp": stamp,
                "source": str(data.get("source", "")),
                "is_hold": bool(data.get("is_hold", False)),
            }
        except Exception as exc:
            self.get_logger().warn(f"目标字段不完整，忽略该帧: {exc}")
            return None

    def workspace_contains(self, x_mm, y_mm, z_mm):
        return (
            float(self.cfg["workspace_x_min_mm"]) <= x_mm <= float(self.cfg["workspace_x_max_mm"])
            and float(self.cfg["workspace_y_min_mm"]) <= y_mm <= float(self.cfg["workspace_y_max_mm"])
            and float(self.cfg["workspace_z_min_mm"]) <= z_mm <= float(self.cfg["workspace_z_max_mm"])
        )

    def suggest_base_adjust(self, snapshot):
        x_mm = snapshot["x_mm"]
        y_mm = snapshot["y_mm"]
        z_mm = snapshot["z_mm"]
        request = {
            "reason": "target_out_of_workspace",
            "target_x_mm": x_mm,
            "target_y_mm": y_mm,
            "target_z_mm": z_mm,
            "workspace_x_min": float(self.cfg["workspace_x_min_mm"]),
            "workspace_x_max": float(self.cfg["workspace_x_max_mm"]),
            "workspace_y_min": float(self.cfg["workspace_y_min_mm"]),
            "workspace_y_max": float(self.cfg["workspace_y_max_mm"]),
            "suggested_action": "stop_and_reobserve",
            "move_distance_mm": 0.0,
            "turn_deg": 0.0,
        }

        if x_mm > float(self.cfg["workspace_x_max_mm"]):
            request["suggested_action"] = "move_forward"
            request["move_distance_mm"] = float(x_mm - float(self.cfg["workspace_x_max_mm"]))
        elif x_mm < float(self.cfg["workspace_x_min_mm"]):
            request["suggested_action"] = "move_backward"
            request["move_distance_mm"] = float(float(self.cfg["workspace_x_min_mm"]) - x_mm)
        elif y_mm > float(self.cfg["workspace_y_max_mm"]):
            request["suggested_action"] = "turn_left"
            request["turn_deg"] = 10.0
        elif y_mm < float(self.cfg["workspace_y_min_mm"]):
            request["suggested_action"] = "turn_right"
            request["turn_deg"] = 10.0
        elif (
            z_mm > float(self.cfg["workspace_z_max_mm"])
            or z_mm < float(self.cfg["workspace_z_min_mm"])
        ):
            request["suggested_action"] = "stop_and_reobserve"

        return request

    def publish_base_adjust_request(self, snapshot):
        if not self.bool_cfg("enable_base_adjust_request"):
            return
        if time.time() - self.base_adjust_sent_at < 0.5:
            return

        request = self.suggest_base_adjust(snapshot)
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.pub_base_adjust.publish(msg)
        self.base_adjust_sent_at = time.time()
        self.get_logger().warn(f"目标超出工作空间，发布底盘调整请求: {msg.data}")

    def try_build_snapshot(self):
        required = int(self.cfg["stable_target_min_frames"])
        if len(self.target_buffer) < required:
            return None

        targets = list(self.target_buffer)[-required:]
        xs = [item["x_mm"] for item in targets]
        ys = [item["y_mm"] for item in targets]
        zs = [item["z_mm"] for item in targets]

        max_std = max(stddev(xs), stddev(ys), stddev(zs))
        if max_std > float(self.cfg["stable_target_max_position_std_mm"]):
            return None

        latest = targets[-1]
        snapshot = {
            "x_mm": mean(xs),
            "y_mm": mean(ys),
            "z_mm": mean(zs),
            "pixel_u": int(round(mean([item["pixel_u"] for item in targets]))),
            "pixel_v": int(round(mean([item["pixel_v"] for item in targets]))),
            "confidence": latest["confidence"],
            "stamp": latest["stamp"],
            "source": latest["source"],
            "is_hold": latest["is_hold"],
            "frozen_at": time.time(),
        }

        if not self.workspace_contains(snapshot["x_mm"], snapshot["y_mm"], snapshot["z_mm"]):
            if (
                float(self.cfg["workspace_x_min_mm"]) <= snapshot["x_mm"] <= float(self.cfg["workspace_x_max_mm"])
                and float(self.cfg["workspace_y_min_mm"]) <= snapshot["y_mm"] <= float(self.cfg["workspace_y_max_mm"])
            ):
                self.get_logger().warn("目标 z 超出工作空间，安全拒绝本次抓取。")
            else:
                self.publish_base_adjust_request(snapshot)
            self.target_buffer.clear()
            return None

        self.get_logger().info(
            "target_snapshot created: "
            f"x={snapshot['x_mm']:.1f}mm y={snapshot['y_mm']:.1f}mm z={snapshot['z_mm']:.1f}mm "
            f"pixel=({snapshot['pixel_u']},{snapshot['pixel_v']}) conf={snapshot['confidence']:.3f}"
        )
        return snapshot

    def snapshot_expired(self):
        if self.current_snapshot is None:
            return True
        age_frozen = time.time() - float(self.current_snapshot["frozen_at"])
        age_target = time.time() - float(self.current_snapshot["stamp"])
        limit = float(self.cfg["stage_target_max_age_s"])
        expired = age_frozen > limit or age_target > limit
        if expired:
            self.get_logger().warn(
                f"snapshot 过期: frozen_age={age_frozen:.2f}s target_age={age_target:.2f}s limit={limit:.2f}s"
            )
        return expired

    def allow_expired_snapshot_in_current_state(self):
        if self.state not in self.motion_states:
            return False
        if not self.bool_cfg("allow_snapshot_expire_during_motion"):
            return False
        self.get_logger().warn(
            "snapshot expired by age but allowed during current motion stage"
        )
        return True

    def clear_snapshot(self):
        self.current_snapshot = None
        self.target_buffer.clear()
        self.done_logged = False

    def to_moveit_xyz(self, x_mm, y_mm, z_mm):
        scale = float(self.cfg["moveit_position_scale"])
        x_m = float(self.cfg["moveit_x_sign"]) * float(x_mm) * scale + float(self.cfg["moveit_x_offset_m"])
        y_m = float(self.cfg["moveit_y_sign"]) * float(y_mm) * scale + float(self.cfg["moveit_y_offset_m"])
        z_m = float(self.cfg["moveit_z_sign"]) * float(z_mm) * scale + float(self.cfg["moveit_z_offset_m"])
        return x_m, y_m, z_m

    def move_line_mm(self, stage_name, x_mm, y_mm, z_mm):
        if not self.workspace_contains(x_mm, y_mm, z_mm):
            self.get_logger().error(
                f"{stage_name} 超出工作空间，拒绝调用 move_line: "
                f"x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm"
            )
            return False

        x_m, y_m, z_m = self.to_moveit_xyz(x_mm, y_mm, z_mm)
        self.get_logger().info(
            f"{stage_name}: mm=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) -> "
            f"moveit({self.cfg['moveit_frame_id']}) m=({x_m:.4f},{y_m:.4f},{z_m:.4f})"
        )

        if self.bool_cfg("dry_run"):
            self.get_logger().info(f"{stage_name}: dry_run=true，仅打印，不调用 /move_line_cmd")
            return True

        if not self.move_line_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/move_line_cmd 服务不可用")
            return False

        req = MoveLineCmd.Request()
        req.x = float(x_m)
        req.y = float(y_m)
        req.z = float(z_m)
        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().error(f"{stage_name}: /move_line_cmd 调用超时")
            return False

        result = future.result()
        if result is None:
            self.get_logger().error(f"{stage_name}: /move_line_cmd 返回空结果")
            return False

        self.get_logger().info(
            f"{stage_name}: /move_line_cmd success={result.success} message={result.message}"
        )
        return bool(result.success)

    def publish_gripper(self, value, stage_name):
        self.get_logger().info(f"{stage_name}: gripper_cmd={float(value):.3f}")
        if not self.bool_cfg("enable_gripper"):
            self.get_logger().info(f"{stage_name}: enable_gripper=false，跳过夹爪发布")
            return True
        if self.bool_cfg("dry_run"):
            self.get_logger().info(f"{stage_name}: dry_run=true，仅打印，不发布 /gripper_cmd")
            return True

        msg = Float32()
        msg.data = float(value)
        self.pub_gripper.publish(msg)
        return True

    def on_timer(self):
        now = time.time()

        if self.state == "IDLE":
            if self.bool_cfg("auto_start"):
                self.set_state("WAIT_TARGET", "auto_start=true")
            return

        if self.state == "WAIT_TARGET":
            if self.current_snapshot is not None:
                if self.snapshot_expired():
                    self.clear_snapshot()
                else:
                    self.set_state("OPEN_GRIPPER", "稳定目标已冻结")
                    return

            snapshot = self.try_build_snapshot()
            if snapshot is not None:
                self.current_snapshot = snapshot
                self.set_state("OPEN_GRIPPER", "target_snapshot created")
            return

        if self.current_snapshot is None:
            self.set_state("RECOVER", "抓取阶段缺少 snapshot")
            return

        if self.snapshot_expired():
            if not self.allow_expired_snapshot_in_current_state():
                self.set_state("RECOVER", "snapshot 已过期")
                return

        target_x = float(self.current_snapshot["x_mm"])
        target_y = float(self.current_snapshot["y_mm"])
        target_z = float(self.current_snapshot["z_mm"])

        pre_grasp_x = target_x + float(self.cfg["grasp_offset_x_mm"])
        pre_grasp_y = target_y + float(self.cfg["grasp_offset_y_mm"])
        pre_grasp_z = target_z + float(self.cfg["pre_grasp_z_offset_mm"])

        grasp_x = target_x + float(self.cfg["grasp_offset_x_mm"])
        grasp_y = target_y + float(self.cfg["grasp_offset_y_mm"])
        grasp_z = target_z + float(self.cfg["grasp_offset_z_mm"])

        lift_x = grasp_x
        lift_y = grasp_y
        lift_z = grasp_z + float(self.cfg["lift_up_mm"])

        if self.state == "OPEN_GRIPPER":
            if not self.state_action_done:
                self.publish_gripper(self.cfg["gripper_open_value"], "OPEN_GRIPPER")
                self.state_action_done = True
            if now - self.state_enter_time >= 1.0:
                self.set_state("MOVE_TO_PRE_GRASP")
            return

        if self.state == "MOVE_TO_PRE_GRASP":
            ok = self.move_line_mm("MOVE_TO_PRE_GRASP", pre_grasp_x, pre_grasp_y, pre_grasp_z)
            self.set_state("MOVE_LINE_TO_GRASP" if ok else "RECOVER", "预抓取位成功" if ok else "预抓取位失败")
            return

        if self.state == "MOVE_LINE_TO_GRASP":
            ok = self.move_line_mm("MOVE_LINE_TO_GRASP", grasp_x, grasp_y, grasp_z)
            self.set_state("CLOSE_GRIPPER" if ok else "RECOVER", "抓取位成功" if ok else "抓取位失败")
            return

        if self.state == "CLOSE_GRIPPER":
            if not self.state_action_done:
                self.publish_gripper(self.cfg["gripper_close_value"], "CLOSE_GRIPPER")
                self.state_action_done = True
            if now - self.state_enter_time >= 1.0:
                self.set_state("LIFT")
            return

        if self.state == "LIFT":
            ok = self.move_line_mm("LIFT", lift_x, lift_y, lift_z)
            self.set_state("DONE" if ok else "RECOVER", "抬升成功" if ok else "抬升失败")
            return

        if self.state == "DONE":
            if not self.done_logged:
                self.get_logger().info("抓取流程完成。当前第一版仅执行单个红块抓取。")
                self.done_logged = True
            self.clear_snapshot()
            return

        if self.state == "RECOVER":
            if not self.state_action_done:
                self.get_logger().error("进入 RECOVER：本轮抓取终止，等待重新观测目标。")
                self.clear_snapshot()
                self.state_action_done = True
            if now - self.state_enter_time >= 1.0:
                self.set_state("WAIT_TARGET", "recover 完成")


def resolve_config_path():
    if len(sys.argv) >= 2 and sys.argv[1].strip():
        return os.path.abspath(sys.argv[1].strip())
    env_path = os.environ.get("DOG_VISUAL_GRASP_CONFIG", "").strip()
    if env_path:
        return os.path.abspath(env_path)
    base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, "config", "grasp_config.yaml")


def main(args=None):
    config_path = resolve_config_path()
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    rclpy.init(args=args)
    node = VisualMoveItGraspNode(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
