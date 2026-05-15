#!/usr/bin/env python3
import json
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera
from red_block_grasp_ros2.core.color_red_block_detector import RedColorBlockDetector
from red_block_grasp_ros2.core.target_localizer import TargetLocalizer
from red_block_grasp_ros2.core.yolo_detector import YoloRedBlockDetector


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
        self.declare_parameter("infer_imgsz", 256)
        self.declare_parameter("max_targets", 2)
        self.declare_parameter("timer_period", 0.08)
        self.declare_parameter("show_window", True)
        self.declare_parameter("display_scale", 0.75)
        self.declare_parameter("detector_device", "")
        self.declare_parameter("detector_half", False)
        self.declare_parameter("perf_log_interval_s", 3.0)

        self.declare_parameter("detector_mode", "fusion")
        self.declare_parameter("yolo_every_n_frames", 5)
        self.declare_parameter("enable_color_detector", True)
        self.declare_parameter("enable_yolo_detector", True)
        self.declare_parameter("color_calib_path", "")
        self.declare_parameter("fusion_iou_threshold", 0.10)
        self.declare_parameter("stable_frame_count", 3)
        self.declare_parameter("stable_position_threshold_mm", 20.0)
        self.declare_parameter("stable_depth_threshold_mm", 30.0)
        self.declare_parameter("publish_only_stable", False)
        self.declare_parameter("target_lock_timeout_s", 1.0)
        self.declare_parameter("fusion_min_color_to_yolo_area_ratio", 0.45)
        self.declare_parameter("fusion_edge_margin_ratio", 0.18)

        self.declare_parameter("color_min_depth_mm", 100.0)
        self.declare_parameter("color_max_depth_mm", 700.0)
        self.declare_parameter("color_min_area", 250.0)
        self.declare_parameter("color_min_area_ratio", 0.0)
        self.declare_parameter("color_max_area_ratio", 0.35)
        self.declare_parameter("color_aspect_min", 0.35)
        self.declare_parameter("color_aspect_max", 3.0)
        self.declare_parameter("color_extent_min", 0.25)
        self.declare_parameter("color_solidity_min", 0.50)
        self.declare_parameter("color_morph_kernel_size", 5)
        self.declare_parameter("color_erode_kernel_size", 5)
        self.declare_parameter("color_hsv_h1_min", 0)
        self.declare_parameter("color_hsv_h1_max", 10)
        self.declare_parameter("color_hsv_h2_min", 170)
        self.declare_parameter("color_hsv_h2_max", 180)
        self.declare_parameter("color_hsv_s_min", 70)
        self.declare_parameter("color_hsv_v_min", 45)
        self.declare_parameter("color_lab_a_min", 145)
        self.declare_parameter("color_bgr_r_min", 80)
        self.declare_parameter("color_bgr_rg_delta", 35)
        self.declare_parameter("color_bgr_rb_delta", 25)
        self.declare_parameter("color_bgr_b_max", 210)

        self.declare_parameter("safe_roi_x_min_ratio", 0.12)
        self.declare_parameter("safe_roi_x_max_ratio", 0.88)
        self.declare_parameter("safe_roi_y_min_ratio", 0.12)
        self.declare_parameter("safe_roi_y_max_ratio", 0.88)
        self.declare_parameter("enable_target_lock", True)
        self.declare_parameter("lock_max_pixel_jump", 160.0)
        self.declare_parameter("center_weight", 0.25)
        self.declare_parameter("base_filter_alpha", 0.55)

        self.declare_parameter("save_hard_samples", False)
        self.declare_parameter("hard_sample_conf_thres", 0.75)
        self.declare_parameter("hard_sample_interval_s", 0.8)
        self.declare_parameter("force_save_samples", False)
        self.declare_parameter("force_sample_interval_s", 1.5)
        self.declare_parameter("hard_sample_dir", "/home/sunrise/dog/ros2_red_block_ws/hard_samples")

        self.model_path = self.get_parameter("model_path").value
        self.handeye_path = self.get_parameter("handeye_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.infer_imgsz = int(self.get_parameter("infer_imgsz").value)
        self.max_targets = int(self.get_parameter("max_targets").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.show_window = self.parse_bool(self.get_parameter("show_window").value)
        self.display_scale = float(self.get_parameter("display_scale").value)
        self.detector_device = str(self.get_parameter("detector_device").value)
        self.detector_half = self.parse_bool(self.get_parameter("detector_half").value)
        self.perf_log_interval_s = float(self.get_parameter("perf_log_interval_s").value)

        self.detector_mode = str(self.get_parameter("detector_mode").value).strip().lower()
        if self.detector_mode not in ("yolo", "color", "fusion"):
            self.detector_mode = "fusion"
        self.yolo_every_n_frames = max(1, int(self.get_parameter("yolo_every_n_frames").value))
        self.enable_color_detector = self.parse_bool(self.get_parameter("enable_color_detector").value)
        self.enable_yolo_detector = self.parse_bool(self.get_parameter("enable_yolo_detector").value)
        self.color_calib_path = str(self.get_parameter("color_calib_path").value).strip()
        self.fusion_iou_threshold = float(self.get_parameter("fusion_iou_threshold").value)
        self.stable_frame_count = max(1, int(self.get_parameter("stable_frame_count").value))
        self.stable_position_threshold_mm = float(self.get_parameter("stable_position_threshold_mm").value)
        self.stable_depth_threshold_mm = float(self.get_parameter("stable_depth_threshold_mm").value)
        self.publish_only_stable = self.parse_bool(self.get_parameter("publish_only_stable").value)
        self.target_lock_timeout_s = float(self.get_parameter("target_lock_timeout_s").value)
        self.fusion_min_color_to_yolo_area_ratio = float(
            self.get_parameter("fusion_min_color_to_yolo_area_ratio").value
        )
        self.fusion_edge_margin_ratio = float(self.get_parameter("fusion_edge_margin_ratio").value)

        self.safe_roi_x_min_ratio = float(self.get_parameter("safe_roi_x_min_ratio").value)
        self.safe_roi_x_max_ratio = float(self.get_parameter("safe_roi_x_max_ratio").value)
        self.safe_roi_y_min_ratio = float(self.get_parameter("safe_roi_y_min_ratio").value)
        self.safe_roi_y_max_ratio = float(self.get_parameter("safe_roi_y_max_ratio").value)
        self.enable_target_lock = self.parse_bool(self.get_parameter("enable_target_lock").value)
        self.lock_max_pixel_jump = float(self.get_parameter("lock_max_pixel_jump").value)
        self.center_weight = float(self.get_parameter("center_weight").value)
        self.base_filter_alpha = float(self.get_parameter("base_filter_alpha").value)

        self.save_hard_samples = self.parse_bool(self.get_parameter("save_hard_samples").value)
        self.hard_sample_conf_thres = float(self.get_parameter("hard_sample_conf_thres").value)
        self.hard_sample_interval_s = float(self.get_parameter("hard_sample_interval_s").value)
        self.force_save_samples = self.parse_bool(self.get_parameter("force_save_samples").value)
        self.force_sample_interval_s = float(self.get_parameter("force_sample_interval_s").value)
        self.hard_sample_dir = self.get_parameter("hard_sample_dir").value
        os.makedirs(self.hard_sample_dir, exist_ok=True)

        self.color_params = self.build_color_params()
        self.apply_color_calib_file(self.color_params)

        self.pub_target = self.create_publisher(String, "/red_block/target_base", 10)
        self.sub_arm_state = self.create_subscription(String, "/roarm_m3/state", self.on_arm_state, 10)

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0
        self.locked_pixel = None
        self.lock_last_seen_time = 0.0
        self.filtered_base = None
        self.stable_window = deque(maxlen=self.stable_frame_count)
        self.frame_index = 0
        self.latest_yolo_detections = []
        self.latest_yolo_time = 0.0
        self.last_hard_sample_time = 0.0
        self.last_force_sample_time = 0.0

        self.camera = OrbbecRgbdCamera()
        self.yolo_detector = None
        if self.enable_yolo_detector and self.detector_mode in ("yolo", "fusion"):
            self.yolo_detector = YoloRedBlockDetector(
                model_path=self.model_path,
                conf_thres=self.conf_thres,
                max_targets=self.max_targets,
                imgsz=self.infer_imgsz,
                device=self.detector_device,
                half=self.detector_half,
            )

        self.color_detector = None
        if self.enable_color_detector and self.detector_mode in ("color", "fusion"):
            self.color_detector = RedColorBlockDetector(
                min_depth_mm=self.color_params["color_min_depth_mm"],
                max_depth_mm=self.color_params["color_max_depth_mm"],
                min_area=self.color_params["color_min_area"],
                min_area_ratio=self.color_params["color_min_area_ratio"],
                max_area_ratio=self.color_params["color_max_area_ratio"],
                aspect_min=self.color_params["color_aspect_min"],
                aspect_max=self.color_params["color_aspect_max"],
                extent_min=self.color_params["color_extent_min"],
                solidity_min=self.color_params["color_solidity_min"],
                morph_kernel_size=self.color_params["color_morph_kernel_size"],
                erode_kernel_size=self.color_params["color_erode_kernel_size"],
                hsv_h1_min=self.color_params["color_hsv_h1_min"],
                hsv_h1_max=self.color_params["color_hsv_h1_max"],
                hsv_h2_min=self.color_params["color_hsv_h2_min"],
                hsv_h2_max=self.color_params["color_hsv_h2_max"],
                hsv_s_min=self.color_params["color_hsv_s_min"],
                hsv_v_min=self.color_params["color_hsv_v_min"],
                lab_a_min=self.color_params["color_lab_a_min"],
                bgr_r_min=self.color_params["color_bgr_r_min"],
                bgr_rg_delta=self.color_params["color_bgr_rg_delta"],
                bgr_rb_delta=self.color_params["color_bgr_rb_delta"],
                bgr_b_max=self.color_params["color_bgr_b_max"],
                max_targets=self.max_targets,
            )
            self.log_color_params()

        self.localizer = TargetLocalizer(
            handeye_path=self.handeye_path,
            depth_roi_radius=6,
            min_depth_mm=100,
            max_depth_mm=700,
        )

        self.window_name = "ROS2 Target Localizer"
        self.last_time = time.time()
        self.last_perf_log_time = 0.0
        self.fps = 0.0

        self.get_logger().info("Starting RGBD camera...")
        self.camera.start()
        if self.yolo_detector is not None:
            self.get_logger().info("Loading YOLO model...")
            self.yolo_detector.load()

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 800, 520)

        self.timer = self.create_timer(self.timer_period, self.on_timer)
        self.get_logger().info(
            f"Target localizer started. mode={self.detector_mode}, yolo_every_n_frames={self.yolo_every_n_frames}"
        )

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        if value is None:
            return False
        text = str(value).strip().lower()
        return text in ("1", "true", "yes", "on")

    def build_color_params(self):
        return {
            "color_hsv_h1_min": int(self.get_parameter("color_hsv_h1_min").value),
            "color_hsv_h1_max": int(self.get_parameter("color_hsv_h1_max").value),
            "color_hsv_h2_min": int(self.get_parameter("color_hsv_h2_min").value),
            "color_hsv_h2_max": int(self.get_parameter("color_hsv_h2_max").value),
            "color_hsv_s_min": int(self.get_parameter("color_hsv_s_min").value),
            "color_hsv_v_min": int(self.get_parameter("color_hsv_v_min").value),
            "color_lab_a_min": int(self.get_parameter("color_lab_a_min").value),
            "color_bgr_r_min": int(self.get_parameter("color_bgr_r_min").value),
            "color_bgr_rg_delta": int(self.get_parameter("color_bgr_rg_delta").value),
            "color_bgr_rb_delta": int(self.get_parameter("color_bgr_rb_delta").value),
            "color_bgr_b_max": int(self.get_parameter("color_bgr_b_max").value),
            "color_min_area": float(self.get_parameter("color_min_area").value),
            "color_min_area_ratio": float(self.get_parameter("color_min_area_ratio").value),
            "color_max_area_ratio": float(self.get_parameter("color_max_area_ratio").value),
            "color_aspect_min": float(self.get_parameter("color_aspect_min").value),
            "color_aspect_max": float(self.get_parameter("color_aspect_max").value),
            "color_extent_min": float(self.get_parameter("color_extent_min").value),
            "color_solidity_min": float(self.get_parameter("color_solidity_min").value),
            "color_morph_kernel_size": int(self.get_parameter("color_morph_kernel_size").value),
            "color_erode_kernel_size": int(self.get_parameter("color_erode_kernel_size").value),
            "color_min_depth_mm": float(self.get_parameter("color_min_depth_mm").value),
            "color_max_depth_mm": float(self.get_parameter("color_max_depth_mm").value),
        }

    def read_calib_file(self, path):
        ext = os.path.splitext(path)[1].lower()
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()

        if ext == ".json":
            return json.loads(text)

        if ext in (".yaml", ".yml"):
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError(
                    "PyYAML is required to read YAML color calibration files. "
                    "Install python3-yaml or provide a JSON file."
                ) from exc
            return yaml.safe_load(text)

        try:
            import yaml
            data = yaml.safe_load(text)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
        return json.loads(text)

    def apply_color_calib_file(self, color_params):
        if not self.color_calib_path:
            return

        if not os.path.exists(self.color_calib_path):
            self.get_logger().warn(
                f"Color calibration file does not exist: {self.color_calib_path}. Using ROS/default color parameters."
            )
            return

        try:
            data = self.read_calib_file(self.color_calib_path)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to load color calibration file {self.color_calib_path}: {exc}. "
                "Using ROS/default color parameters."
            )
            return

        if not isinstance(data, dict):
            self.get_logger().warn(
                f"Color calibration file {self.color_calib_path} did not contain a mapping. "
                "Using ROS/default color parameters."
            )
            return

        loaded_keys = []
        for key in color_params:
            if key not in data:
                continue
            try:
                if isinstance(color_params[key], int):
                    color_params[key] = int(data[key])
                else:
                    color_params[key] = float(data[key])
                loaded_keys.append(key)
            except Exception as exc:
                self.get_logger().warn(f"Ignoring invalid color calibration value for {key}: {exc}")

        self.get_logger().info(
            f"Loaded color calibration from {self.color_calib_path}. Updated keys: {', '.join(loaded_keys)}"
        )

    def log_color_params(self):
        p = self.color_params
        self.get_logger().info(
            "Color detector params: "
            f"H=[{p['color_hsv_h1_min']},{p['color_hsv_h1_max']}]|"
            f"[{p['color_hsv_h2_min']},{p['color_hsv_h2_max']}], "
            f"S_min={p['color_hsv_s_min']}, V_min={p['color_hsv_v_min']}, "
            f"Lab_a_min={p['color_lab_a_min']}, "
            f"R_min={p['color_bgr_r_min']}, "
            f"R-G>={p['color_bgr_rg_delta']}, R-B>={p['color_bgr_rb_delta']}, "
            f"B_max={p['color_bgr_b_max']}, min_area={p['color_min_area']}"
        )

    def on_arm_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Invalid /roarm_m3/state JSON: {e}")
            return
        if not data.get("connected", False) or not data.get("state_valid", False):
            return
        state = data.get("state", None)
        if isinstance(state, dict):
            self.latest_arm_state = state
            self.latest_arm_state_time = time.time()

    def detection_in_safe_roi(self, det, image_width, image_height):
        cx, cy = det.center
        return (
            image_width * self.safe_roi_x_min_ratio <= cx <= image_width * self.safe_roi_x_max_ratio
            and image_height * self.safe_roi_y_min_ratio <= cy <= image_height * self.safe_roi_y_max_ratio
        )

    @staticmethod
    def pixel_distance(p1, p2):
        return math.hypot(float(p1[0]) - float(p2[0]), float(p1[1]) - float(p2[1]))

    @staticmethod
    def bbox_iou(a, b):
        ix1 = max(a.x1, b.x1)
        iy1 = max(a.y1, b.y1)
        ix2 = min(a.x2, b.x2)
        iy2 = min(a.y2, b.y2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        area_a = max(1, (a.x2 - a.x1) * (a.y2 - a.y1))
        area_b = max(1, (b.x2 - b.x1) * (b.y2 - b.y1))
        return float(inter) / float(area_a + area_b - inter)

    @staticmethod
    def bbox_area(det):
        return max(1, int(det.x2 - det.x1) * int(det.y2 - det.y1))

    def clear_target_lock(self):
        # 锁丢失后一起清理滤波状态，避免两个红块的 base 坐标混在同一窗口里。
        self.locked_pixel = None
        self.lock_last_seen_time = 0.0
        self.filtered_base = None
        self.stable_window.clear()

    def target_lock_active(self, now):
        return (
            self.enable_target_lock
            and self.locked_pixel is not None
            and (now - self.lock_last_seen_time) <= self.target_lock_timeout_s
        )

    def select_detection(self, detections, image_width, image_height, now=None):
        if now is None:
            now = time.time()
        if not detections:
            if self.locked_pixel is not None and not self.target_lock_active(now):
                self.clear_target_lock()
            return None, "no_detection"

        safe = [det for det in detections if self.detection_in_safe_roi(det, image_width, image_height)]
        if not safe:
            if self.locked_pixel is not None and not self.target_lock_active(now):
                self.clear_target_lock()
            return None, "no_safe_detection"

        if self.target_lock_active(now):
            nearest = min(safe, key=lambda det: self.pixel_distance(det.center, self.locked_pixel))
            if self.pixel_distance(nearest.center, self.locked_pixel) <= self.lock_max_pixel_jump:
                return nearest, "locked_target"
            return None, "locked_target_wait_timeout"
        elif self.locked_pixel is not None:
            self.clear_target_lock()

        img_cx = image_width / 2.0
        img_cy = image_height / 2.0
        norm = max(1.0, math.hypot(img_cx, img_cy))

        def score(det):
            cx, cy = det.center
            det_score = float(getattr(det, "score", getattr(det, "conf", 0.0)))
            return det_score - self.center_weight * (math.hypot(cx - img_cx, cy - img_cy) / norm)

        return max(safe, key=score), "new_target"

    def annotate_color_with_yolo_support(self, color_detections, yolo_detections):
        for det in color_detections:
            det.debug_info["best_yolo_iou"] = 0.0
            det.debug_info["best_yolo_area_ratio"] = None
            det.debug_info["fusion_small_local"] = False
            det.debug_info["fusion_edge_local"] = False
            if not yolo_detections:
                continue

            best_yolo = max(yolo_detections, key=lambda ydet: self.bbox_iou(det, ydet))
            best_iou = self.bbox_iou(det, best_yolo)
            det.debug_info["best_yolo_iou"] = float(best_iou)
            yolo_area = float(self.bbox_area(best_yolo))
            color_area = float(self.bbox_area(det))
            area_ratio = color_area / max(yolo_area, 1.0)
            yolo_w = max(1, best_yolo.width)
            yolo_h = max(1, best_yolo.height)
            edge_margin_x = self.fusion_edge_margin_ratio * yolo_w
            edge_margin_y = self.fusion_edge_margin_ratio * yolo_h
            cx, cy = det.center
            edge_local = (
                cx < best_yolo.x1 + edge_margin_x
                or cx > best_yolo.x2 - edge_margin_x
                or cy < best_yolo.y1 + edge_margin_y
                or cy > best_yolo.y2 - edge_margin_y
            )
            det.debug_info["best_yolo_area_ratio"] = float(area_ratio)
            det.debug_info["fusion_edge_local"] = bool(edge_local)
            det.debug_info["fusion_small_local"] = bool(
                best_iou >= self.fusion_iou_threshold
                and area_ratio < self.fusion_min_color_to_yolo_area_ratio
                and edge_local
            )

    def select_fusion_detection(self, color_detections, yolo_detections, image_width, image_height, now=None):
        if now is None:
            now = time.time()
        if self.detector_mode == "color":
            selected, reason = self.select_detection(color_detections, image_width, image_height, now=now)
            return selected, reason, "color"

        if self.detector_mode == "yolo":
            selected, reason = self.select_detection(yolo_detections, image_width, image_height, now=now)
            return selected, reason, "yolo"

        self.annotate_color_with_yolo_support(color_detections, yolo_detections)

        preferred_color = [det for det in color_detections if not det.debug_info.get("fusion_small_local", False)]
        if preferred_color:
            selected, reason = self.select_detection(preferred_color, image_width, image_height, now=now)
            if selected is not None:
                return selected, "fusion_color_primary" if reason == "new_target" else reason, "color"

        if color_detections:
            selected, reason = self.select_detection(color_detections, image_width, image_height, now=now)
            if selected is not None and not selected.debug_info.get("fusion_small_local", False):
                return selected, "fusion_color_only" if reason == "new_target" else reason, "color"

        if yolo_detections:
            selected, reason = self.select_detection(yolo_detections, image_width, image_height, now=now)
            if selected is not None and color_detections:
                return selected, "fusion_yolo_color_confirm" if reason == "new_target" else reason, "yolo_assisted"
        if color_detections:
            selected, reason = self.select_detection(color_detections, image_width, image_height, now=now)
            return selected, "fusion_color_local_only" if selected is not None else reason, "color_local"

        # fusion 模式下没有颜色确认时不盲信 YOLO，避免现场光照波动时误抓。
        return None, "fusion_no_color_confirmation", "none"

    def filter_base(self, base):
        current = {key: float(base[key]) for key in ("x", "y", "z")}
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

    def update_stability(self, base_mm, depth_mm):
        self.stable_window.append(
            {
                "x": float(base_mm["x"]),
                "y": float(base_mm["y"]),
                "z": float(base_mm["z"]),
                "depth": float(depth_mm),
            }
        )
        if len(self.stable_window) < self.stable_frame_count:
            return False, len(self.stable_window), dict(base_mm)

        arr = np.array([[p["x"], p["y"], p["z"], p["depth"]] for p in self.stable_window], dtype=np.float64)
        med = np.median(arr, axis=0)
        pos_dist = np.linalg.norm(arr[:, :3] - med[:3], axis=1)
        depth_diff = np.abs(arr[:, 3] - med[3])
        stable = (
            float(np.max(pos_dist)) <= self.stable_position_threshold_mm
            and float(np.max(depth_diff)) <= self.stable_depth_threshold_mm
        )
        stable_base = {"x": float(med[0]), "y": float(med[1]), "z": float(med[2])}
        return stable, len(self.stable_window), stable_base

    def empty_message(self, now, reason, select_reason, image_width=0, image_height=0):
        return {
            "stamp": now,
            "valid": False,
            "stable": False,
            "stable_frames": 0,
            "reason": reason,
            "select_reason": select_reason,
            "detector_mode": self.detector_mode,
            "source": None,
            "target_id": None,
            "confidence": None,
            "score": None,
            "pixel": None,
            "bbox": None,
            "depth_mm": None,
            "camera_mm": None,
            "raw_base_mm": None,
            "base_mm": None,
            "detections_count": 0,
            "color_candidates_count": 0,
            "yolo_candidates_count": 0,
            "image_width": int(image_width),
            "image_height": int(image_height),
            "arm_state_age_s": None,
        }

    def publish_json(self, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_target.publish(msg)

    def on_timer(self):
        frame_start = time.time()
        bgr, depth_mm, camera_matrix = self.camera.read(timeout_ms=100)
        if bgr is None or depth_mm is None or camera_matrix is None:
            return
        read_done = time.time()

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        self.frame_index += 1
        image_height, image_width = bgr.shape[:2]

        color_detections = []
        if self.color_detector is not None:
            color_detections = self.color_detector.detect(bgr, depth_mm=depth_mm, camera_matrix=camera_matrix)
        color_done = time.time()

        yolo_detections = self.latest_yolo_detections
        run_yolo = self.yolo_detector is not None and (
            self.detector_mode == "yolo" or self.frame_index % self.yolo_every_n_frames == 1
        )
        if run_yolo:
            yolo_detections = self.yolo_detector.detect(bgr)
            self.latest_yolo_detections = yolo_detections
            self.latest_yolo_time = now
        infer_done = time.time()

        selected, select_reason, source = self.select_fusion_detection(
            color_detections, yolo_detections, image_width, image_height, now=now
        )

        msg_dict = self.empty_message(now, "", select_reason, image_width, image_height)
        msg_dict.update(
            {
                "source": source,
                "detections_count": len(color_detections) + len(yolo_detections),
                "color_candidates_count": len(color_detections),
                "yolo_candidates_count": len(yolo_detections),
            }
        )
        if self.latest_arm_state is not None:
            msg_dict["arm_state_age_s"] = now - self.latest_arm_state_time

        if selected is None:
            msg_dict["reason"] = select_reason
            self.stable_window.clear()
        elif self.latest_arm_state is None:
            self.locked_pixel = selected.center
            self.lock_last_seen_time = now
            msg_dict["reason"] = "no_arm_state"
            self.stable_window.clear()
        elif now - self.latest_arm_state_time > 1.0:
            self.locked_pixel = selected.center
            self.lock_last_seen_time = now
            msg_dict["reason"] = "arm_state_timeout"
            self.stable_window.clear()
        else:
            self.locked_pixel = selected.center
            self.lock_last_seen_time = now
            location = self.localizer.localize(selected, bgr, depth_mm, camera_matrix, self.latest_arm_state)
            if location is None:
                msg_dict["reason"] = "invalid_depth"
                self.stable_window.clear()
            else:
                raw_base = location["base_mm"]
                filtered_base = self.filter_base(raw_base)
                stable, stable_frames, stable_base = self.update_stability(filtered_base, location["depth_mm"])
                output_base = stable_base if stable else filtered_base
                msg_dict.update(
                    {
                        "valid": True,
                        "stable": bool(stable),
                        "stable_frames": int(stable_frames),
                        "reason": "ok",
                        "target_id": 0,
                        "confidence": float(getattr(selected, "conf", 0.0)),
                        "score": float(getattr(selected, "score", getattr(selected, "conf", 0.0))),
                        "pixel": location["pixel"],
                        "bbox": {
                            "x1": int(selected.x1),
                            "y1": int(selected.y1),
                            "x2": int(selected.x2),
                            "y2": int(selected.y2),
                            "w": int(selected.width),
                            "h": int(selected.height),
                        },
                        "depth_mm": location["depth_mm"],
                        "camera_mm": location["camera_mm"],
                        "raw_base_mm": raw_base,
                        "base_mm": output_base,
                    }
                )

        if not self.publish_only_stable or msg_dict.get("stable", False) or not msg_dict.get("valid", False):
            self.publish_json(msg_dict)
        publish_done = time.time()

        self.maybe_save_samples(bgr, selected, msg_dict)
        display_done = publish_done
        if self.show_window:
            display = self.draw_debug(bgr, color_detections, yolo_detections, selected, msg_dict)
            if 0.05 < self.display_scale < 1.0:
                display = cv2.resize(display, None, fx=self.display_scale, fy=self.display_scale, interpolation=cv2.INTER_AREA)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rclpy.shutdown()
            display_done = time.time()

        if self.perf_log_interval_s > 0 and now - self.last_perf_log_time >= self.perf_log_interval_s:
            self.last_perf_log_time = now
            self.get_logger().info(
                "localizer-perf: "
                f"fps={self.fps:.1f} read_ms={(read_done - frame_start) * 1000.0:.1f} "
                f"color_ms={(color_done - read_done) * 1000.0:.1f} "
                f"yolo_ms={(infer_done - color_done) * 1000.0:.1f} "
                f"publish_ms={(publish_done - infer_done) * 1000.0:.1f} "
                f"display_ms={(display_done - publish_done) * 1000.0:.1f} mode={self.detector_mode}"
            )

    def save_sample_image(self, bgr, selected, msg_dict, tag):
        stamp_ms = int(time.time() * 1000)
        safe_tag = str(tag).replace("/", "_").replace(" ", "_")
        image_path = os.path.join(self.hard_sample_dir, f"sample_{stamp_ms}_{safe_tag}.jpg")
        json_path = os.path.join(self.hard_sample_dir, f"sample_{stamp_ms}_{safe_tag}.json")
        image = bgr.copy()
        if selected is not None:
            cv2.rectangle(image, (selected.x1, selected.y1), (selected.x2, selected.y2), (0, 255, 255), 2)
            cv2.circle(image, selected.center, 5, (0, 255, 255), -1)
        ok = cv2.imwrite(image_path, image)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(msg_dict, f, ensure_ascii=False, indent=2)
        if ok:
            self.get_logger().info(f"Saved sample: {image_path}")

    def maybe_save_samples(self, bgr, selected, msg_dict):
        now = time.time()
        if self.force_save_samples and now - self.last_force_sample_time >= self.force_sample_interval_s:
            self.save_sample_image(bgr, selected, msg_dict, "force")
            self.last_force_sample_time = now
            return
        if not self.save_hard_samples or now - self.last_hard_sample_time < self.hard_sample_interval_s:
            return
        if not msg_dict.get("valid", False):
            self.save_sample_image(bgr, selected, msg_dict, str(msg_dict.get("reason", "invalid")))
            self.last_hard_sample_time = now
            return
        conf = float(msg_dict.get("confidence", 0.0))
        if conf < self.hard_sample_conf_thres:
            self.save_sample_image(bgr, selected, msg_dict, f"lowconf_{conf:.2f}")
            self.last_hard_sample_time = now

    def draw_debug(self, image, color_detections, yolo_detections, selected, msg_dict):
        display = image.copy()
        image_height, image_width = image.shape[:2]
        x_min = int(image_width * self.safe_roi_x_min_ratio)
        x_max = int(image_width * self.safe_roi_x_max_ratio)
        y_min = int(image_height * self.safe_roi_y_min_ratio)
        y_max = int(image_height * self.safe_roi_y_max_ratio)
        cv2.rectangle(display, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)
        cv2.putText(
            display,
            f"Target Localizer | {self.detector_mode} | FPS={self.fps:.1f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
        )
        lock_active = self.target_lock_active(time.time())
        lock_age = 0.0 if self.locked_pixel is None else max(0.0, time.time() - self.lock_last_seen_time)
        locked_pixel_text = "none" if self.locked_pixel is None else f"({self.locked_pixel[0]},{self.locked_pixel[1]})"
        debug_lines = [
            f"LOCKED: {'yes' if lock_active else 'no'}",
            f"lock_age: {lock_age:.2f}s",
            f"locked_pixel: {locked_pixel_text}",
            f"selected_source: {msg_dict.get('source')}",
            f"detector_mode: {self.detector_mode}",
        ]
        for idx, line in enumerate(debug_lines):
            cv2.putText(
                display,
                line,
                (20, 65 + idx * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

        for idx, det in enumerate(yolo_detections):
            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), (255, 0, 0), 2)
            label = f"YOLO conf={float(getattr(det, 'conf', 0.0)):.2f}"
            cv2.putText(
                display,
                label,
                (det.x1, max(18, det.y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 0, 0),
                2,
            )
        for idx, det in enumerate(color_detections):
            is_selected = selected is det
            color = (0, 255, 255) if is_selected else (0, 0, 255)
            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), color, 3 if is_selected else 2)
            cv2.circle(display, det.center, 5, color, -1)
            det_area = float(det.debug_info.get("area", self.bbox_area(det)))
            extent = float(det.debug_info.get("extent", 0.0))
            solidity = float(det.debug_info.get("solidity", 0.0))
            label = (
                f"COLOR score={float(getattr(det, 'score', getattr(det, 'conf', 0.0))):.2f} "
                f"area={det_area:.0f} extent={extent:.2f} solidity={solidity:.2f}"
            )
            if det.debug_info.get("fusion_small_local", False):
                label += " local"
            cv2.putText(
                display,
                label,
                (det.x1, min(image_height - 8, det.y2 + 16 if det.y1 < 24 else det.y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                2,
            )

        if selected is not None and all(selected is not det for det in color_detections):
            cv2.rectangle(display, (selected.x1, selected.y1), (selected.x2, selected.y2), (0, 255, 255), 3)
            cv2.circle(display, selected.center, 5, (0, 255, 255), -1)
            cv2.putText(
                display,
                "SELECTED",
                (selected.x1, max(18, selected.y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                2,
            )
        elif selected is not None:
            cv2.putText(
                display,
                "SELECTED",
                (selected.x1, max(18, selected.y1 - 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                2,
            )

        if msg_dict.get("valid", False):
            base = msg_dict["base_mm"]
            text = (
                f"{msg_dict.get('source')} stable={msg_dict.get('stable')} "
                f"base=({base['x']:.1f},{base['y']:.1f},{base['z']:.1f})"
            )
            cv2.putText(display, text, (20, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2)
            if selected is not None:
                dbg = getattr(selected, "debug_info", {})
                selected_lines = [
                    f"selected_area: {float(dbg.get('area', self.bbox_area(selected))):.0f}",
                    f"selected_score: {float(getattr(selected, 'score', getattr(selected, 'conf', 0.0))):.2f}",
                    f"selected_depth: {float(msg_dict.get('depth_mm', 0.0)):.1f}",
                    f"depth_source: {dbg.get('depth_source', 'window')}",
                    f"color_mean_r: {float(dbg.get('mean_r', 0.0)):.1f}",
                    f"color_rg_delta: {float(dbg.get('mean_rg_delta', 0.0)):.1f}",
                    f"color_rb_delta: {float(dbg.get('mean_rb_delta', 0.0)):.1f}",
                ]
                for idx, line in enumerate(selected_lines):
                    cv2.putText(
                        display,
                        line,
                        (20, 220 + idx * 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 255),
                        2,
                    )
        else:
            cv2.putText(
                display,
                f"invalid: {msg_dict.get('reason')} / {msg_dict.get('select_reason')}",
                (20, 195),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
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
