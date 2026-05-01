#!/usr/bin/env python3
# 中文说明：
# ROS2 目标定位节点。
#
# 当前节点不控制机械臂，只做：
# 1. 打开 RGBD 相机
# 2. YOLO 检测 red_block
# 3. 读取机械臂当前状态
# 4. 用深度 + 手眼标定计算 red#0 的 base 坐标
# 5. 发布 /red_block/target_base

import json
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera
from red_block_grasp_ros2.core.yolo_detector import YoloRedBlockDetector
from red_block_grasp_ros2.core.target_localizer import TargetLocalizer
from red_block_grasp_ros2.roarm_m3 import RoArmM3


class TargetLocalizerNode(Node):
    def __init__(self):
        super().__init__("target_localizer_node")

        self.declare_parameter(
            "model_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt"
        )
        self.declare_parameter(
            "handeye_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/handeye/handeye_cam_to_eef.json"
        )
        self.declare_parameter("arm_port", "/dev/ttyUSB0")
        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("max_targets", 4)
        self.declare_parameter("timer_period", 0.3)
        self.declare_parameter("show_window", True)

        self.model_path = self.get_parameter("model_path").value
        self.handeye_path = self.get_parameter("handeye_path").value
        self.arm_port = self.get_parameter("arm_port").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.max_targets = int(self.get_parameter("max_targets").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.show_window = bool(self.get_parameter("show_window").value)

        self.pub_target = self.create_publisher(String, "/red_block/target_base", 10)

        self.camera = OrbbecRgbdCamera()
        self.detector = YoloRedBlockDetector(
            model_path=self.model_path,
            conf_thres=self.conf_thres,
            max_targets=self.max_targets,
        )
        self.localizer = TargetLocalizer(
            handeye_path=self.handeye_path,
            depth_roi_radius=6,
            min_depth_mm=100,
            max_depth_mm=700,
        )

        self.arm = RoArmM3(port=self.arm_port)

        self.window_name = "ROS2 Target Localizer"
        self.last_time = time.time()
        self.fps = 0.0

        self.get_logger().info("Starting RGBD camera...")
        self.camera.start()

        self.get_logger().info("Loading YOLO model...")
        self.detector.load()

        self.get_logger().info("Connecting RoArm-M3 for state reading...")
        self.arm.connect()

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1100, 700)

        self.timer = self.create_timer(self.timer_period, self.on_timer)

        self.get_logger().info("Target localizer node started.")
        self.get_logger().info(f"model_path = {self.model_path}")
        self.get_logger().info(f"handeye_path = {self.handeye_path}")
        self.get_logger().info(f"arm_port = {self.arm_port}")

    def on_timer(self):
        bgr, depth_mm, camera_matrix = self.camera.read(timeout_ms=100)
        if bgr is None or depth_mm is None or camera_matrix is None:
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        detections = self.detector.detect(bgr)

        arm_state = self.arm.read_latest_state(duration_s=0.2)

        msg_dict = {
            "stamp": now,
            "valid": False,
            "reason": "",
            "target_id": None,
            "confidence": None,
            "pixel": None,
            "depth_mm": None,
            "camera_mm": None,
            "base_mm": None,
            "detections_count": len(detections),
        }

        selected = detections[0] if detections else None
        location = None

        if selected is None:
            msg_dict["reason"] = "no_detection"
        elif arm_state is None:
            msg_dict["reason"] = "no_arm_state"
        else:
            location = self.localizer.localize(
                detection=selected,
                bgr=bgr,
                depth_mm=depth_mm,
                camera_matrix=camera_matrix,
                arm_state=arm_state,
            )

            if location is None:
                msg_dict["reason"] = "invalid_depth"
            else:
                msg_dict["valid"] = True
                msg_dict["reason"] = "ok"
                msg_dict["target_id"] = 0
                msg_dict["confidence"] = float(selected.conf)
                msg_dict["pixel"] = location["pixel"]
                msg_dict["depth_mm"] = location["depth_mm"]
                msg_dict["camera_mm"] = location["camera_mm"]
                msg_dict["base_mm"] = location["base_mm"]

        msg = String()
        msg.data = json.dumps(msg_dict, ensure_ascii=False)
        self.pub_target.publish(msg)

        if self.show_window:
            display = self.draw_debug(bgr, detections, msg_dict)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.get_logger().info("Quit key received.")
                rclpy.shutdown()

    def draw_debug(self, image, detections, msg_dict):
        display = image.copy()

        cv2.putText(
            display,
            f"ROS2 Target Localizer | FPS={self.fps:.1f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        if not detections:
            cv2.putText(
                display,
                "NO RED BLOCK",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            return display

        for idx, det in enumerate(detections):
            color = (0, 255, 255) if idx == 0 else (0, 0, 255)
            thickness = 3 if idx == 0 else 2

            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), color, thickness)
            cx, cy = det.center
            cv2.circle(display, (cx, cy), 5, color, -1)

            label = f"red#{idx} conf={det.conf:.2f}"
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
            text = f"red#0 base=({base['x']:.1f}, {base['y']:.1f}, {base['z']:.1f}) mm"
            cv2.putText(
                display,
                text,
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
        else:
            cv2.putText(
                display,
                f"target invalid: {msg_dict['reason']}",
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
            self.arm.close()
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
