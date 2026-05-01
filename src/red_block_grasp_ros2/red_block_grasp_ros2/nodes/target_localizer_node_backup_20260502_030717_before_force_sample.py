#!/usr/bin/env python3
# 中文说明：
# ROS2 目标定位节点。
#
# 优化点：
# 1. 不再简单选择 YOLO 置信度最高的目标。
# 2. 只选择图像安全区域内的目标，过滤靠边目标。
# 3. 加入目标锁定，避免多个红块之间来回跳。
# 4. 对 base 坐标做 EMA 滤波，减少坐标抖动。
#
# 本节点不连接 /dev/ttyUSB0。
# 机械臂串口只允许 roarm_driver_node 占用。

import json
import math
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera
from red_block_grasp_ros2.core.yolo_detector import YoloRedBlockDetector
from red_block_grasp_ros2.core.target_localizer import TargetLocalizer


class TargetLocalizerNode(Node):
    def __init__(self):
        super().__init__("target_localizer_node")

        self.declare_parameter(
            "model_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt",
        )
        self.declare_parameter(
            "handeye_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/handeye/handeye_cam_to_eef.json",
        )

        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("infer_imgsz", 320)
        self.declare_parameter("max_targets", 4)
        self.declare_parameter("timer_period", 0.3)
        self.declare_parameter("show_window", True)

        self.declare_parameter("safe_roi_x_min_ratio", 0.12)
        self.declare_parameter("safe_roi_x_max_ratio", 0.88)
        self.declare_parameter("safe_roi_y_min_ratio", 0.12)
        self.declare_parameter("safe_roi_y_max_ratio", 0.88)

        self.declare_parameter("enable_target_lock", True)
        self.declare_parameter("lock_max_pixel_jump", 160.0)
        self.declare_parameter("center_weight", 0.25)

        self.declare_parameter("base_filter_alpha", 0.55)

        self.declare_parameter("save_hard_samples", True)
        self.declare_parameter("hard_sample_conf_thres", 0.65)
        self.declare_parameter("hard_sample_interval_s", 1.0)
        self.declare_parameter("hard_sample_dir", "/home/sunrise/dog/ros2_red_block_ws/hard_samples")

        self.model_path = self.get_parameter("model_path").value
        self.handeye_path = self.get_parameter("handeye_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.infer_imgsz = int(self.get_parameter("infer_imgsz").value)
        self.max_targets = int(self.get_parameter("max_targets").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.show_window = bool(self.get_parameter("show_window").value)

        self.safe_roi_x_min_ratio = float(self.get_parameter("safe_roi_x_min_ratio").value)
        self.safe_roi_x_max_ratio = float(self.get_parameter("safe_roi_x_max_ratio").value)
        self.safe_roi_y_min_ratio = float(self.get_parameter("safe_roi_y_min_ratio").value)
        self.safe_roi_y_max_ratio = float(self.get_parameter("safe_roi_y_max_ratio").value)

        self.enable_target_lock = bool(self.get_parameter("enable_target_lock").value)
        self.lock_max_pixel_jump = float(self.get_parameter("lock_max_pixel_jump").value)
        self.center_weight = float(self.get_parameter("center_weight").value)

        self.base_filter_alpha = float(self.get_parameter("base_filter_alpha").value)

        self.save_hard_samples = bool(self.get_parameter("save_hard_samples").value)
        self.hard_sample_conf_thres = float(self.get_parameter("hard_sample_conf_thres").value)
        self.hard_sample_interval_s = float(self.get_parameter("hard_sample_interval_s").value)
        self.hard_sample_dir = self.get_parameter("hard_sample_dir").value
        self.last_hard_sample_time = 0.0
        os.makedirs(self.hard_sample_dir, exist_ok=True)

        self.pub_target = self.create_publisher(String, "/red_block/target_base", 10)
        self.sub_arm_state = self.create_subscription(
            String,
            "/roarm_m3/state",
            self.on_arm_state,
            10,
        )

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0

        self.locked_pixel = None
        self.filtered_base = None

        self.camera = OrbbecRgbdCamera()
        self.detector = YoloRedBlockDetector(
            model_path=self.model_path,
            conf_thres=self.conf_thres,
            max_targets=self.max_targets,
            imgsz=self.infer_imgsz,
        )
        self.localizer = TargetLocalizer(
            handeye_path=self.handeye_path,
            depth_roi_radius=6,
            min_depth_mm=100,
            max_depth_mm=700,
        )

        self.window_name = "ROS2 Target Localizer"
        self.last_time = time.time()
        self.fps = 0.0

        self.get_logger().info("Starting RGBD camera...")
        self.camera.start()

        self.get_logger().info("Loading YOLO model...")
        self.detector.load()

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1100, 700)

        self.timer = self.create_timer(self.timer_period, self.on_timer)

        self.get_logger().info("Target localizer node started.")
        self.get_logger().info("This node does not use /dev/ttyUSB0.")
        self.get_logger().info("Target lock + safe ROI + base EMA filter enabled.")

    def on_arm_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Invalid /roarm_m3/state JSON: {e}")
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

    def detection_in_safe_roi(self, det, image_width, image_height):
        cx, cy = det.center

        x_min = image_width * self.safe_roi_x_min_ratio
        x_max = image_width * self.safe_roi_x_max_ratio
        y_min = image_height * self.safe_roi_y_min_ratio
        y_max = image_height * self.safe_roi_y_max_ratio

        return x_min <= cx <= x_max and y_min <= cy <= y_max

    @staticmethod
    def pixel_distance(p1, p2):
        return math.hypot(float(p1[0]) - float(p2[0]), float(p1[1]) - float(p2[1]))

    def select_detection(self, detections, image_width, image_height):
        if not detections:
            self.locked_pixel = None
            self.filtered_base = None
            return None, "no_detection"

        safe_detections = [
            det for det in detections
            if self.detection_in_safe_roi(det, image_width, image_height)
        ]

        if not safe_detections:
            self.locked_pixel = None
            self.filtered_base = None
            return None, "no_safe_detection"

        if self.enable_target_lock and self.locked_pixel is not None:
            nearest = min(
                safe_detections,
                key=lambda det: self.pixel_distance(det.center, self.locked_pixel),
            )
            dist = self.pixel_distance(nearest.center, self.locked_pixel)

            if dist <= self.lock_max_pixel_jump:
                return nearest, "locked_target"

        img_cx = image_width / 2.0
        img_cy = image_height / 2.0
        norm = math.hypot(img_cx, img_cy)

        def score(det):
            cx, cy = det.center
            dist_norm = math.hypot(cx - img_cx, cy - img_cy) / max(norm, 1.0)
            return float(det.conf) - self.center_weight * dist_norm

        selected = max(safe_detections, key=score)
        return selected, "new_target"

    def filter_base(self, base):
        current = {
            "x": float(base["x"]),
            "y": float(base["y"]),
            "z": float(base["z"]),
        }

        if self.filtered_base is None:
            self.filtered_base = dict(current)
            return dict(current)

        a = self.base_filter_alpha
        self.filtered_base = {
            "x": a * current["x"] + (1.0 - a) * self.filtered_base["x"],
            "y": a * current["y"] + (1.0 - a) * self.filtered_base["y"],
            "z": a * current["z"] + (1.0 - a) * self.filtered_base["z"],
        }

        return dict(self.filtered_base)

    def on_timer(self):
        bgr, depth_mm, camera_matrix = self.camera.read(timeout_ms=100)
        if bgr is None or depth_mm is None or camera_matrix is None:
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        image_height, image_width = bgr.shape[:2]
        detections = self.detector.detect(bgr)

        selected, select_reason = self.select_detection(detections, image_width, image_height)

        msg_dict = {
            "stamp": now,
            "valid": False,
            "reason": "",
            "select_reason": select_reason,
            "target_id": None,
            "confidence": None,
            "pixel": None,
            "depth_mm": None,
            "camera_mm": None,
            "raw_base_mm": None,
            "base_mm": None,
            "detections_count": len(detections),
            "image_width": int(image_width),
            "image_height": int(image_height),
            "arm_state_age_s": None,
        }

        if self.latest_arm_state is not None:
            msg_dict["arm_state_age_s"] = now - self.latest_arm_state_time

        if selected is None:
            msg_dict["reason"] = select_reason

        elif self.latest_arm_state is None:
            msg_dict["reason"] = "no_arm_state"

        elif now - self.latest_arm_state_time > 1.0:
            msg_dict["reason"] = "arm_state_timeout"

        else:
            location = self.localizer.localize(
                detection=selected,
                bgr=bgr,
                depth_mm=depth_mm,
                camera_matrix=camera_matrix,
                arm_state=self.latest_arm_state,
            )

            if location is None:
                msg_dict["reason"] = "invalid_depth"
            else:
                raw_base = location["base_mm"]
                filtered_base = self.filter_base(raw_base)

                cx, cy = selected.center
                self.locked_pixel = (cx, cy)

                msg_dict["valid"] = True
                msg_dict["reason"] = "ok"
                msg_dict["target_id"] = 0
                msg_dict["confidence"] = float(selected.conf)
                msg_dict["pixel"] = location["pixel"]
                msg_dict["depth_mm"] = location["depth_mm"]
                msg_dict["camera_mm"] = location["camera_mm"]
                msg_dict["raw_base_mm"] = raw_base
                msg_dict["base_mm"] = filtered_base

        msg = String()
        msg.data = json.dumps(msg_dict, ensure_ascii=False)
        self.pub_target.publish(msg)

        self.maybe_save_hard_sample(bgr, selected, msg_dict)

        if self.show_window:
            display = self.draw_debug(bgr, detections, selected, msg_dict)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.get_logger().info("Quit key received.")
                rclpy.shutdown()


    def maybe_save_hard_sample(self, bgr, selected, msg_dict):
        if not self.save_hard_samples:
            return

        now = time.time()
        if now - self.last_hard_sample_time < self.hard_sample_interval_s:
            return

        need_save = False

        if not msg_dict.get("valid", False):
            need_save = True
            tag = str(msg_dict.get("reason", "invalid"))
        else:
            conf = float(msg_dict.get("confidence", 0.0))
            if conf < self.hard_sample_conf_thres:
                need_save = True
                tag = f"lowconf_{conf:.2f}"
            else:
                tag = "normal"

        if not need_save:
            return

        filename = f"hard_{int(now * 1000)}_{tag}.jpg"
        path = os.path.join(self.hard_sample_dir, filename)

        image = bgr.copy()
        if selected is not None:
            cv2.rectangle(image, (selected.x1, selected.y1), (selected.x2, selected.y2), (0, 255, 255), 2)
            cx, cy = selected.center
            cv2.circle(image, (cx, cy), 5, (0, 255, 255), -1)

        cv2.imwrite(path, image)
        self.last_hard_sample_time = now
        self.get_logger().info(f"Saved hard sample: {path}")

    def draw_debug(self, image, detections, selected, msg_dict):
        display = image.copy()
        image_height, image_width = image.shape[:2]

        x_min = int(image_width * self.safe_roi_x_min_ratio)
        x_max = int(image_width * self.safe_roi_x_max_ratio)
        y_min = int(image_height * self.safe_roi_y_min_ratio)
        y_max = int(image_height * self.safe_roi_y_max_ratio)
        cv2.rectangle(display, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)

        cv2.putText(
            display,
            f"ROS2 Target Localizer | FPS={self.fps:.1f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        for det in detections:
            is_selected = selected is det
            color = (0, 255, 255) if is_selected else (0, 0, 255)
            thickness = 3 if is_selected else 2

            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), color, thickness)
            cx, cy = det.center
            cv2.circle(display, (cx, cy), 5, color, -1)

            label = f"red conf={det.conf:.2f}"
            if is_selected:
                label = f"selected conf={det.conf:.2f}"

            cv2.putText(
                display,
                label,
                (det.x1, max(25, det.y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        if msg_dict["valid"]:
            base = msg_dict["base_mm"]
            text = f"base=({base['x']:.1f}, {base['y']:.1f}, {base['z']:.1f}) mm"
            cv2.putText(display, text, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(
                display,
                f"invalid: {msg_dict['reason']} / {msg_dict['select_reason']}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        return display

    def destroy_node(self):
        try:
            self.camera.stop()
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizerNode()

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
