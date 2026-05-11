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
        self.declare_parameter("fusion_iou_threshold", 0.10)
        self.declare_parameter("stable_frame_count", 3)
        self.declare_parameter("stable_position_threshold_mm", 20.0)
        self.declare_parameter("stable_depth_threshold_mm", 30.0)
        self.declare_parameter("publish_only_stable", False)

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
        self.fusion_iou_threshold = float(self.get_parameter("fusion_iou_threshold").value)
        self.stable_frame_count = max(1, int(self.get_parameter("stable_frame_count").value))
        self.stable_position_threshold_mm = float(self.get_parameter("stable_position_threshold_mm").value)
        self.stable_depth_threshold_mm = float(self.get_parameter("stable_depth_threshold_mm").value)
        self.publish_only_stable = self.parse_bool(self.get_parameter("publish_only_stable").value)

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

        self.pub_target = self.create_publisher(String, "/red_block/target_base", 10)
        self.sub_arm_state = self.create_subscription(String, "/roarm_m3/state", self.on_arm_state, 10)

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0
        self.locked_pixel = None
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
                min_depth_mm=float(self.get_parameter("color_min_depth_mm").value),
                max_depth_mm=float(self.get_parameter("color_max_depth_mm").value),
                min_area=float(self.get_parameter("color_min_area").value),
                min_area_ratio=float(self.get_parameter("color_min_area_ratio").value),
                max_area_ratio=float(self.get_parameter("color_max_area_ratio").value),
                aspect_min=float(self.get_parameter("color_aspect_min").value),
                aspect_max=float(self.get_parameter("color_aspect_max").value),
                extent_min=float(self.get_parameter("color_extent_min").value),
                solidity_min=float(self.get_parameter("color_solidity_min").value),
                morph_kernel_size=int(self.get_parameter("color_morph_kernel_size").value),
                erode_kernel_size=int(self.get_parameter("color_erode_kernel_size").value),
                hsv_s_min=int(self.get_parameter("color_hsv_s_min").value),
                hsv_v_min=int(self.get_parameter("color_hsv_v_min").value),
                lab_a_min=int(self.get_parameter("color_lab_a_min").value),
                bgr_r_min=int(self.get_parameter("color_bgr_r_min").value),
                bgr_rg_delta=int(self.get_parameter("color_bgr_rg_delta").value),
                bgr_rb_delta=int(self.get_parameter("color_bgr_rb_delta").value),
                bgr_b_max=int(self.get_parameter("color_bgr_b_max").value),
                max_targets=self.max_targets,
            )

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

    def select_detection(self, detections, image_width, image_height):
        if not detections:
            self.locked_pixel = None
            self.filtered_base = None
            return None, "no_detection"

        safe = [det for det in detections if self.detection_in_safe_roi(det, image_width, image_height)]
        if not safe:
            self.locked_pixel = None
            self.filtered_base = None
            return None, "no_safe_detection"

        if self.enable_target_lock and self.locked_pixel is not None:
            nearest = min(safe, key=lambda det: self.pixel_distance(det.center, self.locked_pixel))
            if self.pixel_distance(nearest.center, self.locked_pixel) <= self.lock_max_pixel_jump:
                return nearest, "locked_target"

        img_cx = image_width / 2.0
        img_cy = image_height / 2.0
        norm = max(1.0, math.hypot(img_cx, img_cy))

        def score(det):
            cx, cy = det.center
            det_score = float(getattr(det, "score", getattr(det, "conf", 0.0)))
            return det_score - self.center_weight * (math.hypot(cx - img_cx, cy - img_cy) / norm)

        return max(safe, key=score), "new_target"

    def select_fusion_detection(self, color_detections, yolo_detections, image_width, image_height):
        if self.detector_mode == "color":
            selected, reason = self.select_detection(color_detections, image_width, image_height)
            return selected, reason, "color"

        if self.detector_mode == "yolo":
            selected, reason = self.select_detection(yolo_detections, image_width, image_height)
            return selected, reason, "yolo"

        if color_detections and yolo_detections:
            matched = []
            for cdet in color_detections:
                best_iou = max(self.bbox_iou(cdet, ydet) for ydet in yolo_detections)
                if best_iou >= self.fusion_iou_threshold:
                    matched.append((cdet, best_iou))
            if matched:
                matched.sort(key=lambda item: float(getattr(item[0], "score", item[0].conf)) + item[1], reverse=True)
                selected, reason = self.select_detection([item[0] for item in matched], image_width, image_height)
                return selected, "fusion_color_yolo_iou" if selected is not None else reason, "fusion"

        if color_detections:
            selected, reason = self.select_detection(color_detections, image_width, image_height)
            return selected, "fusion_color_only" if selected is not None else reason, "color"

        selected, reason = self.select_detection(yolo_detections, image_width, image_height)
        return selected, "fusion_yolo_fallback" if selected is not None else reason, "yolo_fallback"

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
            color_detections, yolo_detections, image_width, image_height
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
            msg_dict["reason"] = "no_arm_state"
            self.stable_window.clear()
        elif now - self.latest_arm_state_time > 1.0:
            msg_dict["reason"] = "arm_state_timeout"
            self.stable_window.clear()
        else:
            location = self.localizer.localize(selected, bgr, depth_mm, camera_matrix, self.latest_arm_state)
            if location is None:
                msg_dict["reason"] = "invalid_depth"
                self.stable_window.clear()
            else:
                raw_base = location["base_mm"]
                filtered_base = self.filter_base(raw_base)
                stable, stable_frames, stable_base = self.update_stability(filtered_base, location["depth_mm"])
                output_base = stable_base if stable else filtered_base
                cx, cy = selected.center
                self.locked_pixel = (cx, cy)

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

        for det in yolo_detections:
            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), (255, 0, 0), 2)
        for det in color_detections:
            is_selected = selected is det
            color = (0, 255, 255) if is_selected else (0, 0, 255)
            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), color, 3 if is_selected else 2)
            cv2.circle(display, det.center, 5, color, -1)

        if selected is not None and all(selected is not det for det in color_detections):
            cv2.rectangle(display, (selected.x1, selected.y1), (selected.x2, selected.y2), (0, 255, 255), 3)
            cv2.circle(display, selected.center, 5, (0, 255, 255), -1)

        if msg_dict.get("valid", False):
            base = msg_dict["base_mm"]
            text = (
                f"{msg_dict.get('source')} stable={msg_dict.get('stable')} "
                f"base=({base['x']:.1f},{base['y']:.1f},{base['z']:.1f})"
            )
            cv2.putText(display, text, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        else:
            cv2.putText(
                display,
                f"invalid: {msg_dict.get('reason')} / {msg_dict.get('select_reason')}",
                (20, 75),
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
