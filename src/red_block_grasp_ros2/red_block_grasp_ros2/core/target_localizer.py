#!/usr/bin/env python3
# 中文说明：
# 目标定位模块。
# 输入：
#   YOLO 检测框中心像素
#   深度图
#   相机内参
#   机械臂当前末端姿态
#   手眼标定 T_eef_camera
# 输出：
#   红色物料在 camera 坐标系和 base 坐标系下的位置，单位 mm

import json
import math

import numpy as np


def rot_x(a):
    ca = math.cos(a)
    sa = math.sin(a)
    return np.array(
        [
            [1, 0, 0],
            [0, ca, -sa],
            [0, sa, ca],
        ],
        dtype=np.float64,
    )


def rot_y(a):
    ca = math.cos(a)
    sa = math.sin(a)
    return np.array(
        [
            [ca, 0, sa],
            [0, 1, 0],
            [-sa, 0, ca],
        ],
        dtype=np.float64,
    )


def rot_z(a):
    ca = math.cos(a)
    sa = math.sin(a)
    return np.array(
        [
            [ca, -sa, 0],
            [sa, ca, 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )


def rpy_to_R(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def make_T(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def transform_point(T, p):
    p4 = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)
    out = T @ p4
    return out[:3]


def load_handeye(handeye_path):
    with open(handeye_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "T_eef_camera" not in data:
        raise KeyError("handeye file must contain T_eef_camera")

    return np.array(data["T_eef_camera"], dtype=np.float64)


def get_base_to_eef(arm_state):
    x = float(arm_state.get("x", 0.0))
    y = float(arm_state.get("y", 0.0))
    z = float(arm_state.get("z", 0.0))

    roll = float(arm_state.get("r", 0.0))
    pitch = float(arm_state.get("tit", arm_state.get("t", 0.0)))
    yaw = float(arm_state.get("b", 0.0))

    R_base_eef = rpy_to_R(roll, pitch, yaw)
    return make_T(R_base_eef, [x, y, z])


def get_median_depth(depth_mm, u_color, v_color, color_shape, radius, min_depth_mm, max_depth_mm):
    color_h, color_w = color_shape[:2]
    depth_h, depth_w = depth_mm.shape[:2]

    u_depth = int(round(u_color * depth_w / color_w))
    v_depth = int(round(v_color * depth_h / color_h))

    u0 = max(0, u_depth - radius)
    u1 = min(depth_w, u_depth + radius + 1)
    v0 = max(0, v_depth - radius)
    v1 = min(depth_h, v_depth + radius + 1)

    roi = depth_mm[v0:v1, u0:u1]
    valid = roi[(roi > min_depth_mm) & (roi < max_depth_mm)]

    if valid.size == 0:
        return None

    return float(np.median(valid))


def pixel_depth_to_camera_xyz(u, v, depth_mm, camera_matrix):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    z = float(depth_mm)
    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy

    return np.array([x, y, z], dtype=np.float64)


class TargetLocalizer:
    def __init__(
        self,
        handeye_path,
        depth_roi_radius=6,
        min_depth_mm=100,
        max_depth_mm=700,
    ):
        self.T_eef_camera = load_handeye(handeye_path)
        self.depth_roi_radius = int(depth_roi_radius)
        self.min_depth_mm = float(min_depth_mm)
        self.max_depth_mm = float(max_depth_mm)

    def localize(self, detection, bgr, depth_mm, camera_matrix, arm_state):
        cx, cy = detection.center

        depth = get_median_depth(
            depth_mm=depth_mm,
            u_color=cx,
            v_color=cy,
            color_shape=bgr.shape,
            radius=self.depth_roi_radius,
            min_depth_mm=self.min_depth_mm,
            max_depth_mm=self.max_depth_mm,
        )

        if depth is None:
            return None

        p_camera = pixel_depth_to_camera_xyz(cx, cy, depth, camera_matrix)

        T_base_eef = get_base_to_eef(arm_state)
        T_base_camera = T_base_eef @ self.T_eef_camera
        p_base = transform_point(T_base_camera, p_camera)

        return {
            "pixel": {
                "x": int(cx),
                "y": int(cy),
            },
            "depth_mm": float(depth),
            "camera_mm": {
                "x": float(p_camera[0]),
                "y": float(p_camera[1]),
                "z": float(p_camera[2]),
            },
            "base_mm": {
                "x": float(p_base[0]),
                "y": float(p_base[1]),
                "z": float(p_base[2]),
            },
        }
