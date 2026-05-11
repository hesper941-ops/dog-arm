#!/usr/bin/env python3
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class RedColorDetection:
    class_id: int
    class_name: str
    conf: float
    x1: int
    y1: int
    x2: int
    y2: int
    contour: object = None
    mask: object = None
    depth_mm: float = None
    valid_depth: bool = False
    score: float = 0.0
    debug_info: dict = field(default_factory=dict)

    @property
    def center(self):
        return int((self.x1 + self.x2) / 2), int((self.y1 + self.y2) / 2)

    @property
    def width(self):
        return int(self.x2 - self.x1)

    @property
    def height(self):
        return int(self.y2 - self.y1)


class RedColorBlockDetector:
    def __init__(
        self,
        min_depth_mm=100,
        max_depth_mm=700,
        min_area=250,
        min_area_ratio=0.0,
        max_area_ratio=0.35,
        min_width=8,
        min_height=8,
        border_margin_px=3,
        aspect_min=0.35,
        aspect_max=3.0,
        extent_min=0.25,
        solidity_min=0.50,
        circularity_min=0.0,
        morph_kernel_size=5,
        erode_kernel_size=5,
        hsv_h1_min=0,
        hsv_h1_max=10,
        hsv_h2_min=170,
        hsv_h2_max=180,
        hsv_s_min=70,
        hsv_v_min=45,
        lab_a_min=145,
        bgr_r_min=80,
        bgr_rg_delta=35,
        bgr_rb_delta=25,
        bgr_b_max=210,
        max_targets=4,
    ):
        self.min_depth_mm = float(min_depth_mm)
        self.max_depth_mm = float(max_depth_mm)
        self.min_area = float(min_area)
        self.min_area_ratio = float(min_area_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.min_width = int(min_width)
        self.min_height = int(min_height)
        self.border_margin_px = int(border_margin_px)
        self.aspect_min = float(aspect_min)
        self.aspect_max = float(aspect_max)
        self.extent_min = float(extent_min)
        self.solidity_min = float(solidity_min)
        self.circularity_min = float(circularity_min)
        self.morph_kernel_size = int(max(3, morph_kernel_size))
        self.erode_kernel_size = int(max(3, erode_kernel_size))
        self.hsv_h1_min = int(hsv_h1_min)
        self.hsv_h1_max = int(hsv_h1_max)
        self.hsv_h2_min = int(hsv_h2_min)
        self.hsv_h2_max = int(hsv_h2_max)
        self.hsv_s_min = int(hsv_s_min)
        self.hsv_v_min = int(hsv_v_min)
        self.lab_a_min = int(lab_a_min)
        self.bgr_r_min = int(bgr_r_min)
        self.bgr_rg_delta = int(bgr_rg_delta)
        self.bgr_rb_delta = int(bgr_rb_delta)
        self.bgr_b_max = int(bgr_b_max)
        self.max_targets = int(max_targets)

    @staticmethod
    def _odd(value):
        value = int(max(1, value))
        return value if value % 2 == 1 else value + 1

    def build_mask(self, bgr_image):
        blurred = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

        hsv_low = cv2.inRange(
            hsv,
            np.array([self.hsv_h1_min, self.hsv_s_min, self.hsv_v_min], dtype=np.uint8),
            np.array([self.hsv_h1_max, 255, 255], dtype=np.uint8),
        )
        hsv_high = cv2.inRange(
            hsv,
            np.array([self.hsv_h2_min, self.hsv_s_min, self.hsv_v_min], dtype=np.uint8),
            np.array([self.hsv_h2_max, 255, 255], dtype=np.uint8),
        )
        hsv_mask = cv2.bitwise_or(hsv_low, hsv_high)

        lab_a = lab[:, :, 1]
        lab_mask = (lab_a >= self.lab_a_min).astype(np.uint8) * 255

        b = blurred[:, :, 0].astype(np.int16)
        g = blurred[:, :, 1].astype(np.int16)
        r = blurred[:, :, 2].astype(np.int16)
        bgr_rule = (
            (r >= self.bgr_r_min)
            & (r >= g + self.bgr_rg_delta)
            & (r >= b + self.bgr_rb_delta)
            & (b <= self.bgr_b_max)
        )
        bgr_mask = bgr_rule.astype(np.uint8) * 255

        aux_mask = cv2.bitwise_or(lab_mask, bgr_mask)
        mask = cv2.bitwise_and(hsv_mask, aux_mask)
        mask = cv2.medianBlur(mask, self._odd(self.morph_kernel_size))

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self._odd(self.morph_kernel_size), self._odd(self.morph_kernel_size)),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _scale_mask_to_depth(self, mask, depth_shape):
        depth_h, depth_w = depth_shape[:2]
        mask_h, mask_w = mask.shape[:2]
        if (depth_h, depth_w) == (mask_h, mask_w):
            return mask
        return cv2.resize(mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)

    def _valid_depths(self, values):
        values = values[(values > self.min_depth_mm) & (values < self.max_depth_mm)]
        if values.size == 0:
            return values
        median = float(np.median(values))
        gate = max(80.0, median * 0.10)
        return values[np.abs(values.astype(np.float32) - median) <= gate]

    def _window_depth(self, depth_mm, center, image_shape, radius):
        color_h, color_w = image_shape[:2]
        depth_h, depth_w = depth_mm.shape[:2]
        u, v = center
        u_depth = int(round(float(u) * depth_w / max(color_w, 1)))
        v_depth = int(round(float(v) * depth_h / max(color_h, 1)))

        u0 = max(0, u_depth - radius)
        u1 = min(depth_w, u_depth + radius + 1)
        v0 = max(0, v_depth - radius)
        v1 = min(depth_h, v_depth + radius + 1)
        if u1 <= u0 or v1 <= v0:
            return None

        valid = self._valid_depths(depth_mm[v0:v1, u0:u1])
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def robust_depth_from_mask(self, depth_mm, candidate_mask, center, image_shape):
        if depth_mm is None:
            return None, False, {"depth_source": "none", "valid_depth_count": 0}

        scaled_mask = self._scale_mask_to_depth(candidate_mask, depth_mm.shape)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self._odd(self.erode_kernel_size), self._odd(self.erode_kernel_size)),
        )
        eroded = cv2.erode(scaled_mask, kernel, iterations=1)
        if int(np.count_nonzero(eroded)) < 8:
            eroded = scaled_mask

        values = depth_mm[eroded > 0]
        valid = self._valid_depths(values)
        if valid.size >= 8:
            return float(np.median(valid)), True, {
                "depth_source": "mask",
                "valid_depth_count": int(valid.size),
            }

        for radius in (3, 5, 7):
            depth = self._window_depth(depth_mm, center, image_shape, radius)
            if depth is not None:
                return depth, True, {
                    "depth_source": f"center_window_{radius}",
                    "valid_depth_count": int(valid.size),
                }

        return None, False, {
            "depth_source": "invalid",
            "valid_depth_count": int(valid.size),
        }

    def _candidate_score(self, area, area_limit, color_ratio, extent, solidity, depth_valid):
        area_score = min(1.0, float(area) / max(float(area_limit) * 0.18, 1.0))
        color_score = min(1.0, max(0.0, float(color_ratio)))
        shape_score = 0.5 * min(1.0, float(extent)) + 0.5 * min(1.0, float(solidity))
        depth_score = 1.0 if depth_valid else 0.2
        return 0.35 * area_score + 0.30 * color_score + 0.25 * shape_score + 0.10 * depth_score

    def detect(self, bgr_image, depth_mm=None, camera_matrix=None):
        del camera_matrix
        if bgr_image is None:
            return []

        image_h, image_w = bgr_image.shape[:2]
        image_area = max(1, image_w * image_h)
        min_area = max(self.min_area, image_area * self.min_area_ratio)
        max_area = image_area * self.max_area_ratio
        mask = self.build_mask(bgr_image)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < self.min_width or h < self.min_height:
                continue
            if (
                x <= self.border_margin_px
                or y <= self.border_margin_px
                or x + w >= image_w - self.border_margin_px
                or y + h >= image_h - self.border_margin_px
            ):
                continue

            aspect = float(w) / max(float(h), 1.0)
            if aspect < self.aspect_min or aspect > self.aspect_max:
                continue

            bbox_area = float(w * h)
            extent = area / max(bbox_area, 1.0)
            if extent < self.extent_min:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / max(hull_area, 1.0)
            if solidity < self.solidity_min:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            circularity = 0.0
            if perimeter > 1e-6:
                circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if self.circularity_min > 0.0 and circularity < self.circularity_min:
                continue

            candidate_mask = np.zeros((image_h, image_w), dtype=np.uint8)
            cv2.drawContours(candidate_mask, [contour], -1, 255, thickness=-1)
            color_ratio = float(np.count_nonzero(cv2.bitwise_and(mask, candidate_mask))) / max(area, 1.0)
            center = (int(x + w / 2), int(y + h / 2))
            depth, valid_depth, depth_info = self.robust_depth_from_mask(
                depth_mm=depth_mm,
                candidate_mask=candidate_mask,
                center=center,
                image_shape=bgr_image.shape,
            )

            score = self._candidate_score(area, max_area, color_ratio, extent, solidity, valid_depth)
            detections.append(
                RedColorDetection(
                    class_id=0,
                    class_name="red_block",
                    conf=float(score),
                    x1=int(x),
                    y1=int(y),
                    x2=int(x + w),
                    y2=int(y + h),
                    contour=contour,
                    mask=candidate_mask,
                    depth_mm=depth,
                    valid_depth=bool(valid_depth),
                    score=float(score),
                    debug_info={
                        "area": area,
                        "aspect": aspect,
                        "extent": extent,
                        "solidity": solidity,
                        "circularity": circularity,
                        "color_ratio": color_ratio,
                        **depth_info,
                    },
                )
            )

        detections.sort(key=lambda item: item.score, reverse=True)
        return detections[: self.max_targets]
