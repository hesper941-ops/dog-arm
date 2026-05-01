#!/usr/bin/env python3
# 中文说明：
# 视觉闭环任务节点。
#
# 本节点不直接连接相机，也不直接连接机械臂串口。
#
# 订阅：
#   /roarm_m3/state
#   /red_block/target_base
#
# 发布：
#   /roarm_m3/cmd
#   /red_block/visual_servo_state
#
# 核心思想：
#   不再“一次识别后直接运动到目标点”。
#   而是：
#       看一帧目标
#       只移动一小步
#       停一下
#       重新识别
#       再移动一小步
#
# 这样适合眼在手上的相机结构，可以减少机械臂运动时目标突然跑出视野的问题。

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisualServoTaskNode(Node):
    def __init__(self):
        super().__init__("visual_servo_task_node")

        # ===== 基础控制 =====
        self.declare_parameter("auto_start", True)
        self.declare_parameter("move_once_only", True)

        # ===== 初始姿态 =====
        self.declare_parameter("init_b_deg", 0.0)
        self.declare_parameter("init_s_deg", 0.0)
        self.declare_parameter("init_e_deg", 70.0)
        self.declare_parameter("init_t_deg", 90.0)
        self.declare_parameter("init_r_deg", -90.0)
        self.declare_parameter("init_speed_deg_s", 35.0)
        self.declare_parameter("init_acc", 35.0)
        self.declare_parameter("initial_wait_s", 3.0)

        # ===== 找不到目标时的 b 角搜索 =====
        self.declare_parameter("enable_b_scan", True)
        self.declare_parameter("b_scan_offsets", "0,-8,8,-16,16,-24,24")
        self.declare_parameter("scan_timeout_s", 8.0)
        self.declare_parameter("after_scan_pose_wait_s", 2.5)

        # ===== 视觉闭环移动参数 =====
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("max_step_mm", 25.0)
        self.declare_parameter("edge_step_mm", 12.0)
        self.declare_parameter("move_speed", 0.10)
        self.declare_parameter("step_wait_s", 2.0)

        # ===== E 关节视野保持 =====
        # 如果红块在图像上下方向偏离目标像素点，就优先微调 e 关节，避免目标跑出视野。
        self.declare_parameter("enable_e_pixel_servo", True)
        self.declare_parameter("desired_pixel_v", 240.0)
        self.declare_parameter("pixel_v_deadband", 45.0)
        self.declare_parameter("e_kp_deg_per_px", 0.025)
        self.declare_parameter("e_max_step_deg", 3.0)
        self.declare_parameter("e_min_deg", 30.0)
        self.declare_parameter("e_max_deg", 120.0)
        self.declare_parameter("e_pixel_sign", 1.0)
        self.declare_parameter("e_servo_speed_deg_s", 18.0)
        self.declare_parameter("e_servo_acc", 18.0)

        # ===== 到达判定 =====
        self.declare_parameter("target_xy_tolerance_mm", 25.0)
        self.declare_parameter("target_z_tolerance_mm", 25.0)

        # ===== 目标上方点 =====
        self.declare_parameter("safe_above_offset_mm", 120.0)
        self.declare_parameter("grasp_offset_x_mm", 0.0)
        self.declare_parameter("grasp_offset_y_mm", 0.0)
        self.declare_parameter("grasp_offset_z_mm", 0.0)
        self.declare_parameter("min_safe_z_mm", 30.0)

        # ===== 图像安全区域 =====
        # 如果红块中心太靠边，只允许更小步运动，防止冲出视野。
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("safe_roi_x_min_ratio", 0.20)
        self.declare_parameter("safe_roi_x_max_ratio", 0.80)
        self.declare_parameter("safe_roi_y_min_ratio", 0.20)
        self.declare_parameter("safe_roi_y_max_ratio", 0.80)

        # ===== 工作空间保护 =====
        self.declare_parameter("base_x_min", 80.0)
        self.declare_parameter("base_x_max", 700.0)
        self.declare_parameter("base_y_min", -450.0)
        self.declare_parameter("base_y_max", 450.0)
        self.declare_parameter("base_z_min", -30.0)
        self.declare_parameter("base_z_max", 380.0)

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.move_once_only = bool(self.get_parameter("move_once_only").value)

        self.init_pose = {
            "b": float(self.get_parameter("init_b_deg").value),
            "s": float(self.get_parameter("init_s_deg").value),
            "e": float(self.get_parameter("init_e_deg").value),
            "t": float(self.get_parameter("init_t_deg").value),
            "r": float(self.get_parameter("init_r_deg").value),
            "g": None,
        }
        self.init_speed_deg_s = float(self.get_parameter("init_speed_deg_s").value)
        self.init_acc = float(self.get_parameter("init_acc").value)
        self.initial_wait_s = float(self.get_parameter("initial_wait_s").value)

        self.enable_b_scan = bool(self.get_parameter("enable_b_scan").value)
        self.b_scan_offsets = self.parse_offsets(self.get_parameter("b_scan_offsets").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.after_scan_pose_wait_s = float(self.get_parameter("after_scan_pose_wait_s").value)

        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.max_step_mm = float(self.get_parameter("max_step_mm").value)
        self.edge_step_mm = float(self.get_parameter("edge_step_mm").value)
        self.move_speed = float(self.get_parameter("move_speed").value)
        self.step_wait_s = float(self.get_parameter("step_wait_s").value)

        self.enable_e_pixel_servo = bool(self.get_parameter("enable_e_pixel_servo").value)
        self.desired_pixel_v = float(self.get_parameter("desired_pixel_v").value)
        self.pixel_v_deadband = float(self.get_parameter("pixel_v_deadband").value)
        self.e_kp_deg_per_px = float(self.get_parameter("e_kp_deg_per_px").value)
        self.e_max_step_deg = float(self.get_parameter("e_max_step_deg").value)
        self.e_min_deg = float(self.get_parameter("e_min_deg").value)
        self.e_max_deg = float(self.get_parameter("e_max_deg").value)
        self.e_pixel_sign = float(self.get_parameter("e_pixel_sign").value)
        self.e_servo_speed_deg_s = float(self.get_parameter("e_servo_speed_deg_s").value)
        self.e_servo_acc = float(self.get_parameter("e_servo_acc").value)

        self.target_xy_tolerance_mm = float(self.get_parameter("target_xy_tolerance_mm").value)
        self.target_z_tolerance_mm = float(self.get_parameter("target_z_tolerance_mm").value)

        self.safe_above_offset_mm = float(self.get_parameter("safe_above_offset_mm").value)
        self.grasp_offset_x_mm = float(self.get_parameter("grasp_offset_x_mm").value)
        self.grasp_offset_y_mm = float(self.get_parameter("grasp_offset_y_mm").value)
        self.grasp_offset_z_mm = float(self.get_parameter("grasp_offset_z_mm").value)
        self.min_safe_z_mm = float(self.get_parameter("min_safe_z_mm").value)

        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.safe_roi_x_min_ratio = float(self.get_parameter("safe_roi_x_min_ratio").value)
        self.safe_roi_x_max_ratio = float(self.get_parameter("safe_roi_x_max_ratio").value)
        self.safe_roi_y_min_ratio = float(self.get_parameter("safe_roi_y_min_ratio").value)
        self.safe_roi_y_max_ratio = float(self.get_parameter("safe_roi_y_max_ratio").value)

        self.base_x_min = float(self.get_parameter("base_x_min").value)
        self.base_x_max = float(self.get_parameter("base_x_max").value)
        self.base_y_min = float(self.get_parameter("base_y_min").value)
        self.base_y_max = float(self.get_parameter("base_y_max").value)
        self.base_z_min = float(self.get_parameter("base_z_min").value)
        self.base_z_max = float(self.get_parameter("base_z_max").value)

        self.pub_cmd = self.create_publisher(String, "/roarm_m3/cmd", 10)
        self.pub_state = self.create_publisher(String, "/red_block/visual_servo_state", 10)

        self.sub_arm_state = self.create_subscription(
            String,
            "/roarm_m3/state",
            self.on_arm_state,
            10,
        )
        self.sub_target = self.create_subscription(
            String,
            "/red_block/target_base",
            self.on_target,
            10,
        )

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0

        self.latest_target = None
        self.latest_target_time = 0.0

        self.state = "INIT"
        self.started = False
        self.done = False
        self.busy = False

        self.scan_index = 0
        self.last_scan_switch_time = time.time()
        self.state_enter_time = time.time()
        self.last_command_time = 0.0

        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info("Visual servo task node started.")
        self.get_logger().info("This node only publishes /roarm_m3/cmd. It does not use camera or serial directly.")

    @staticmethod
    def parse_offsets(text):
        result = []
        for item in str(text).split(","):
            item = item.strip()
            if item:
                result.append(float(item))
        return result if result else [0.0]

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_cmd.publish(msg)
        self.get_logger().info("PUB /roarm_m3/cmd: " + msg.data)

    def publish_state(self, extra=None):
        now = time.time()

        data = {
            "stamp": now,
            "state": self.state,
            "started": self.started,
            "done": self.done,
            "busy": self.busy,
            "scan_index": self.scan_index,
            "arm_state_age_s": None,
            "target_age_s": None,
        }

        if self.latest_arm_state is not None:
            data["arm_state_age_s"] = now - self.latest_arm_state_time

        if self.latest_target is not None:
            data["target_age_s"] = now - self.latest_target_time

        if extra:
            data.update(extra)

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_state.publish(msg)

    def on_arm_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if not data.get("connected", False):
            return

        if not data.get("state_valid", False):
            return

        state = data.get("state", None)
        if not isinstance(state, dict):
            return

        self.latest_arm_state = state
        self.latest_arm_state_time = time.time()

    def on_target(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if not data.get("valid", False):
            return

        base = data.get("base_mm", None)
        pixel = data.get("pixel", None)

        if not isinstance(base, dict):
            return

        if not isinstance(pixel, dict):
            return

        self.latest_target = data
        self.latest_target_time = time.time()

    def target_is_fresh(self):
        if self.latest_target is None:
            return False
        return time.time() - self.latest_target_time <= self.target_timeout_s

    def arm_state_is_fresh(self):
        if self.latest_arm_state is None:
            return False
        return time.time() - self.latest_arm_state_time <= 1.0

    def enter_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f"State {self.state} -> {new_state}")
            self.state = new_state
            self.state_enter_time = time.time()

    def send_initial_pose(self, b_offset_deg=0.0):
        target = dict(self.init_pose)
        target["b"] = float(self.init_pose["b"]) + float(b_offset_deg)

        cmd = {
            "type": "set_initial_pose",
            "targets_deg": target,
            "move_order": ["b", "t", "r", "s", "e"],
            "speed": self.init_speed_deg_s,
            "acc": self.init_acc,
        }

        self.publish_cmd(cmd)

    def get_pose_from_arm_state(self):
        if self.latest_arm_state is None:
            return None

        s = self.latest_arm_state

        x = float(s.get("x", 0.0))
        y = float(s.get("y", 0.0))
        z = float(s.get("z", 0.0))

        t = self.get_pose_angle_rad(s, ["tit", "pose_t", "tool_t", "t"], default=1.20)
        r = self.get_pose_angle_rad(s, ["rol", "pose_r", "tool_r", "r"], default=-1.57)
        g = self.get_pose_angle_rad(s, ["g", "pose_g", "tool_g"], default=3.14)

        return {
            "x": x,
            "y": y,
            "z": z,
            "t": t,
            "r": r,
            "g": g,
        }

    @staticmethod
    def get_pose_angle_rad(state, keys, default):
        for key in keys:
            if key not in state:
                continue
            try:
                value = float(state[key])
            except Exception:
                continue

            if abs(value) > 6.5:
                return math.radians(value)

            return value

        return float(default)

    def target_pixel_is_safe(self, target):
        pixel = target.get("pixel", {})
        u = float(pixel.get("x", self.image_width / 2))
        v = float(pixel.get("y", self.image_height / 2))

        x_min = self.image_width * self.safe_roi_x_min_ratio
        x_max = self.image_width * self.safe_roi_x_max_ratio
        y_min = self.image_height * self.safe_roi_y_min_ratio
        y_max = self.image_height * self.safe_roi_y_max_ratio

        return x_min <= u <= x_max and y_min <= v <= y_max

    def build_target_eef(self, target_base):
        target_x = float(target_base["x"]) + self.grasp_offset_x_mm
        target_y = float(target_base["y"]) + self.grasp_offset_y_mm
        target_z = max(
            float(target_base["z"]) + self.safe_above_offset_mm + self.grasp_offset_z_mm,
            self.min_safe_z_mm,
        )

        ok, reason = self.check_target_range(target_x, target_y, target_z)
        if not ok:
            raise RuntimeError(reason)

        return {
            "x": target_x,
            "y": target_y,
            "z": target_z,
        }

    def check_target_range(self, x, y, z):
        if not (self.base_x_min <= x <= self.base_x_max):
            return False, f"x out of range: {x:.1f}"
        if not (self.base_y_min <= y <= self.base_y_max):
            return False, f"y out of range: {y:.1f}"
        if not (self.base_z_min <= z <= self.base_z_max):
            return False, f"z out of range: {z:.1f}"
        return True, "ok"

    @staticmethod
    def dist_xy(a, b):
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"])

    def compute_step_target(self, current_pose, target_eef, max_step):
        dx = target_eef["x"] - current_pose["x"]
        dy = target_eef["y"] - current_pose["y"]
        dz = target_eef["z"] - current_pose["z"]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist <= max_step:
            return dict(target_eef), dist

        ratio = max_step / max(dist, 1e-6)

        return {
            "x": current_pose["x"] + dx * ratio,
            "y": current_pose["y"] + dy * ratio,
            "z": current_pose["z"] + dz * ratio,
        }, dist

    def publish_move_pose(self, step_target, pose, label):
        cmd = {
            "type": "move_pose",
            "x": float(step_target["x"]),
            "y": float(step_target["y"]),
            "z": float(step_target["z"]),
            "t": float(pose["t"]),
            "r": float(pose["r"]),
            "g": float(pose["g"]),
            "speed": self.move_speed,
            "label": label,
        }

        self.publish_cmd(cmd)


    def get_current_e_deg(self):
        if self.latest_arm_state is None:
            return None

        # RoArm 原始状态里 e 通常是弧度；部分脚本打印的是角度。
        # 这里做兼容：绝对值小于 6.5 当弧度处理，否则当角度处理。
        try:
            e_value = float(self.latest_arm_state.get("e", 0.0))
        except Exception:
            return None

        if abs(e_value) <= 6.5:
            return math.degrees(e_value)

        return e_value

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def publish_e_joint_correction(self, target_e_deg, reason):
        cmd = {
            "type": "move_joint",
            "joint": 3,
            "angle": float(target_e_deg),
            "speed": self.e_servo_speed_deg_s,
            "acc": self.e_servo_acc,
            "label": reason,
        }
        self.publish_cmd(cmd)

    def should_servo_e_by_pixel(self):
        if not self.enable_e_pixel_servo:
            return False, None

        if self.latest_target is None:
            return False, None

        pixel = self.latest_target.get("pixel", None)
        if not isinstance(pixel, dict):
            return False, None

        try:
            current_v = float(pixel["y"])
        except Exception:
            return False, None

        err_v = self.desired_pixel_v - current_v

        if abs(err_v) <= self.pixel_v_deadband:
            return False, {
                "err_v": err_v,
                "current_v": current_v,
            }

        current_e_deg = self.get_current_e_deg()
        if current_e_deg is None:
            return False, None

        delta_e = self.e_pixel_sign * self.e_kp_deg_per_px * err_v
        delta_e = self.clamp(delta_e, -self.e_max_step_deg, self.e_max_step_deg)

        target_e_deg = current_e_deg + delta_e
        target_e_deg = self.clamp(target_e_deg, self.e_min_deg, self.e_max_deg)

        info = {
            "current_v": current_v,
            "desired_v": self.desired_pixel_v,
            "err_v": err_v,
            "current_e_deg": current_e_deg,
            "delta_e_deg": delta_e,
            "target_e_deg": target_e_deg,
        }

        return True, info

    def do_visual_servo_step(self):
        if not self.target_is_fresh():
            self.enter_state("WAIT_TARGET")
            return

        if not self.arm_state_is_fresh():
            self.get_logger().warn("No fresh arm state. Wait.")
            return

        need_e_servo, e_info = self.should_servo_e_by_pixel()
        if need_e_servo:
            self.get_logger().info(
                "E pixel servo: "
                f"v={e_info['current_v']:.1f}, desired={e_info['desired_v']:.1f}, "
                f"err={e_info['err_v']:.1f}, "
                f"e={e_info['current_e_deg']:.1f} -> {e_info['target_e_deg']:.1f}, "
                f"delta={e_info['delta_e_deg']:.2f}"
            )
            self.publish_e_joint_correction(e_info["target_e_deg"], "e-pixel-servo")
            self.last_command_time = time.time()
            self.busy = True
            self.enter_state("WAIT_AFTER_STEP")
            return

        pose = self.get_pose_from_arm_state()
        target_base = self.latest_target["base_mm"]
        target_eef = self.build_target_eef(target_base)

        xy_error = self.dist_xy(pose, target_eef)
        z_error = abs(pose["z"] - target_eef["z"])

        if xy_error <= self.target_xy_tolerance_mm and z_error <= self.target_z_tolerance_mm:
            self.get_logger().info("Visual servo reached pre-grasp target.")
            self.done = True
            self.enter_state("DONE")
            self.publish_state(
                {
                    "result": "reached_pre_grasp",
                    "target_eef": target_eef,
                    "xy_error": xy_error,
                    "z_error": z_error,
                }
            )
            return

        pixel_safe = self.target_pixel_is_safe(self.latest_target)

        if pixel_safe:
            max_step = self.max_step_mm
            label = "visual-servo-step"
        else:
            max_step = self.edge_step_mm
            label = "edge-safe-step"

        step_target, full_dist = self.compute_step_target(pose, target_eef, max_step)

        self.get_logger().info(
            f"{label}: current=({pose['x']:.1f},{pose['y']:.1f},{pose['z']:.1f}) "
            f"target=({target_eef['x']:.1f},{target_eef['y']:.1f},{target_eef['z']:.1f}) "
            f"step=({step_target['x']:.1f},{step_target['y']:.1f},{step_target['z']:.1f}) "
            f"full_dist={full_dist:.1f} pixel_safe={pixel_safe}"
        )

        self.publish_move_pose(step_target, pose, label)
        self.last_command_time = time.time()
        self.busy = True
        self.enter_state("WAIT_AFTER_STEP")

    def on_timer(self):
        self.publish_state()

        if not self.auto_start:
            return

        now = time.time()

        if self.state == "INIT":
            self.get_logger().info("INIT: send initial pose.")
            self.send_initial_pose(b_offset_deg=0.0)
            self.scan_index = 0
            self.started = True
            self.last_scan_switch_time = now
            self.enter_state("WAIT_INITIAL")
            return

        if self.state == "WAIT_INITIAL":
            if now - self.state_enter_time >= self.initial_wait_s:
                self.enter_state("WAIT_TARGET")
            return

        if self.state == "WAIT_TARGET":
            if self.done and self.move_once_only:
                self.enter_state("DONE")
                return

            if self.target_is_fresh():
                self.enter_state("SERVO_STEP")
                return

            if self.enable_b_scan and now - self.last_scan_switch_time > self.scan_timeout_s:
                self.scan_index += 1

                if self.scan_index >= len(self.b_scan_offsets):
                    self.get_logger().warn("All b scan views tried. Task failed.")
                    self.enter_state("FAIL")
                    return

                offset = self.b_scan_offsets[self.scan_index]
                self.get_logger().info(f"No target. Scan next b offset: {offset:.1f} deg")
                self.send_initial_pose(b_offset_deg=offset)
                self.last_scan_switch_time = now + self.after_scan_pose_wait_s
                self.enter_state("WAIT_INITIAL")
                return

        if self.state == "SERVO_STEP":
            self.do_visual_servo_step()
            return

        if self.state == "WAIT_AFTER_STEP":
            if now - self.last_command_time >= self.step_wait_s:
                self.busy = False

                if self.target_is_fresh():
                    self.enter_state("SERVO_STEP")
                else:
                    self.get_logger().warn("Target lost after step. Stop and wait/search.")
                    self.enter_state("WAIT_TARGET")
            return

        if self.state == "DONE":
            return

        if self.state == "FAIL":
            return


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoTaskNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
