#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "red_block_grasp_ros2"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera


DEFAULT_OUTPUT = "/tmp/red_color_calib_competition.yaml"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="交互式标定红色/绿色/背景阈值。")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--min-depth-mm", type=float, default=100.0)
    parser.add_argument("--max-depth-mm", type=float, default=700.0)
    return parser.parse_args(argv)


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float32), q))


def clamp_int(value, low, high):
    return int(max(low, min(high, round(float(value)))))


class RoiCollector:
    def __init__(self):
        self.dragging = False
        self.start = None
        self.end = None
        self.rect = None

    def on_mouse(self, event, x, y, flags, param):
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start = (x, y)
            self.end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            self.end = (x, y)
            self.rect = self.normalized_rect(self.start, self.end)

    @staticmethod
    def normalized_rect(p1, p2):
        if p1 is None or p2 is None:
            return None
        x1 = min(p1[0], p2[0])
        y1 = min(p1[1], p2[1])
        x2 = max(p1[0], p2[0])
        y2 = max(p1[1], p2[1])
        if x2 - x1 < 6 or y2 - y1 < 6:
            return None
        return x1, y1, x2, y2


def extract_pixels(bgr, rect):
    if rect is None:
        return None
    x1, y1, x2, y2 = rect
    roi = bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return None
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
    b = blur[:, :, 0].reshape(-1).astype(np.float32)
    g = blur[:, :, 1].reshape(-1).astype(np.float32)
    r = blur[:, :, 2].reshape(-1).astype(np.float32)
    h = hsv[:, :, 0].reshape(-1).astype(np.float32)
    s = hsv[:, :, 1].reshape(-1).astype(np.float32)
    v = hsv[:, :, 2].reshape(-1).astype(np.float32)
    a = lab[:, :, 1].reshape(-1).astype(np.float32)
    return {
        "b": b,
        "g": g,
        "r": r,
        "h": h,
        "s": s,
        "v": v,
        "a": a,
        "rg": r - g,
        "rb": r - b,
        "area": float((x2 - x1) * (y2 - y1)),
    }


def split_red_hue(h_values):
    hue = np.asarray(h_values, dtype=np.float32)
    low = hue[hue <= 30.0]
    high = hue[hue >= 150.0]
    return low, high


def build_config(red_pos, green_neg, background_neg):
    neg = green_neg + background_neg

    red_h_low, red_h_high = split_red_hue(red_pos["h"])
    hsv_h1_min = clamp_int(percentile(red_h_low if red_h_low.size > 0 else [0], 5) - 3, 0, 20)
    hsv_h1_max = clamp_int(percentile(red_h_low if red_h_low.size > 0 else [12], 95) + 3, 0, 25)
    hsv_h2_min = clamp_int(percentile(red_h_high if red_h_high.size > 0 else [168], 5) - 3, 150, 180)
    hsv_h2_max = clamp_int(percentile(red_h_high if red_h_high.size > 0 else [180], 95) + 2, 160, 180)

    s_min_pos = percentile(red_pos["s"], 8) - 12
    v_min_pos = percentile(red_pos["v"], 8) - 12
    a_min_pos = percentile(red_pos["a"], 8) - 8
    r_min_pos = percentile(red_pos["r"], 8) - 10
    rg_min_pos = percentile(red_pos["rg"], 8) - 8
    rb_min_pos = percentile(red_pos["rb"], 8) - 8
    b_max_pos = percentile(red_pos["b"], 95) + 20

    if neg["a"].size > 0:
        a_min_pos = max(a_min_pos, percentile(neg["a"], 95) + 4)
    if neg["rg"].size > 0:
        rg_min_pos = max(rg_min_pos, percentile(neg["rg"], 95) + 4)
    if neg["rb"].size > 0:
        rb_min_pos = max(rb_min_pos, percentile(neg["rb"], 95) + 4)
    if background_neg["s"].size > 0:
        s_min_pos = max(s_min_pos, percentile(background_neg["s"], 85))
    if background_neg["v"].size > 0:
        v_min_pos = max(v_min_pos, percentile(background_neg["v"], 15) - 5)
    if background_neg["r"].size > 0:
        r_min_pos = max(r_min_pos, percentile(background_neg["r"], 90))

    min_area = max(120.0, percentile(red_pos["area"], 15) * 0.45)

    return {
        "color_hsv_h1_min": hsv_h1_min,
        "color_hsv_h1_max": hsv_h1_max,
        "color_hsv_h2_min": hsv_h2_min,
        "color_hsv_h2_max": hsv_h2_max,
        "color_hsv_s_min": clamp_int(s_min_pos, 35, 255),
        "color_hsv_v_min": clamp_int(v_min_pos, 30, 255),
        "color_lab_a_min": clamp_int(a_min_pos, 120, 255),
        "color_bgr_r_min": clamp_int(r_min_pos, 45, 255),
        "color_bgr_rg_delta": clamp_int(rg_min_pos, 8, 255),
        "color_bgr_rb_delta": clamp_int(rb_min_pos, 8, 255),
        "color_bgr_b_max": clamp_int(b_max_pos, 0, 255),
        "color_min_valid_depth_count": 8,
        "color_min_area": float(min_area),
        "color_min_area_ratio": 0.0,
        "color_max_area_ratio": 0.35,
        "color_aspect_min": 0.35,
        "color_aspect_max": 3.0,
        "color_extent_min": 0.25,
        "color_solidity_min": 0.50,
        "color_morph_kernel_size": 5,
        "color_erode_kernel_size": 5,
        "color_relative_area_min_ratio": 0.25,
        "color_locked_replace_min_area_ratio": 0.30,
        "color_min_depth_mm": 100.0,
        "color_max_depth_mm": 700.0,
    }


def merge_samples(sample_list):
    if not sample_list:
        return {key: np.asarray([], dtype=np.float32) for key in ("b", "g", "r", "h", "s", "v", "a", "rg", "rb", "area")}
    merged = {}
    for key in sample_list[0]:
        merged[key] = np.concatenate([item[key].reshape(-1) for item in sample_list], axis=0).astype(np.float32)
    return merged


def write_yaml(path, config):
    import yaml

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=False)


def draw_overlay(frame, collector, mode_name, counts, continuous_capture):
    display = frame.copy()
    if collector.rect is not None:
        x1, y1, x2, y2 = collector.rect
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 255), 2)
    if collector.dragging and collector.start is not None and collector.end is not None:
        rect = collector.normalized_rect(collector.start, collector.end)
        if rect is not None:
            x1, y1, x2, y2 = rect
            cv2.rectangle(display, (x1, y1), (x2, y2), (255, 255, 0), 1)

    lines = [
        f"mode: {mode_name}",
        f"capture: {'on' if continuous_capture else 'off'}",
        f"red_positive: {counts['red_positive']}",
        f"green_negative: {counts['green_negative']}",
        f"background_negative: {counts['background_negative']}",
        "keys: 1/2/3 switch mode, c capture once, space toggle capture, s save, q quit",
    ]
    for idx, text in enumerate(lines):
        cv2.putText(
            display,
            text,
            (20, 30 + idx * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
    return display


def main(argv=None):
    args = parse_args(argv)
    camera = OrbbecRgbdCamera()
    collector = RoiCollector()
    window_name = "red_green_threshold_calibration"
    mode = "red_positive"
    continuous_capture = False
    target_samples = max(12, int(args.frames))
    samples = {
        "red_positive": [],
        "green_negative": [],
        "background_negative": [],
    }
    last_capture_time = 0.0

    try:
        camera.start()
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        cv2.setMouseCallback(window_name, collector.on_mouse)

        while True:
            bgr, depth_mm, _ = camera.read(timeout_ms=100)
            if bgr is None or depth_mm is None:
                continue

            now = time.time()
            if continuous_capture and collector.rect is not None and now - last_capture_time >= 0.06:
                sample = extract_pixels(bgr, collector.rect)
                if sample is not None:
                    samples[mode].append(sample)
                    last_capture_time = now

            counts = {key: len(value) for key, value in samples.items()}
            display = draw_overlay(bgr, collector, mode, counts, continuous_capture)
            if args.show or True:
                cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("1"):
                mode = "red_positive"
            elif key == ord("2"):
                mode = "green_negative"
            elif key == ord("3"):
                mode = "background_negative"
            elif key == ord("c"):
                sample = extract_pixels(bgr, collector.rect)
                if sample is not None:
                    samples[mode].append(sample)
            elif key == ord(" "):
                continuous_capture = not continuous_capture
            elif key == ord("r"):
                samples[mode].clear()
            elif key == ord("s"):
                break
            elif key in (ord("q"), 27):
                return 1

            if len(samples["red_positive"]) >= target_samples and len(samples["green_negative"]) >= 8 and len(samples["background_negative"]) >= 8:
                break

        if len(samples["red_positive"]) < 8:
            print("ERROR: red_positive 样本不足，至少需要 8 次采样。")
            return 1
        if len(samples["green_negative"]) < 4:
            print("ERROR: green_negative 样本不足，至少需要 4 次采样。")
            return 1
        if len(samples["background_negative"]) < 4:
            print("ERROR: background_negative 样本不足，至少需要 4 次采样。")
            return 1

        red_pos = merge_samples(samples["red_positive"])
        green_neg = merge_samples(samples["green_negative"])
        background_neg = merge_samples(samples["background_negative"])
        config = build_config(red_pos, green_neg, background_neg)
        config["color_min_depth_mm"] = float(args.min_depth_mm)
        config["color_max_depth_mm"] = float(args.max_depth_mm)
        write_yaml(args.output, config)

        print(f"Saved calibration YAML: {args.output}")
        print("建议采样说明：")
        print("- red_positive 请覆盖红块两个面、不同朝向、轻微阴影和高亮。")
        print("- green_negative 请覆盖绿色物块正面与侧面。")
        print("- background_negative 请覆盖桌面、箱子、阴影和反光区域。")
        print("推荐参数：")
        for key, value in config.items():
            print(f"{key}: {value}")
        return 0
    finally:
        try:
            camera.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
