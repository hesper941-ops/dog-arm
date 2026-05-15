#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    handeye_path = LaunchConfiguration("handeye_path")
    arm_port = LaunchConfiguration("arm_port")
    show_window = LaunchConfiguration("show_window")
    enable_fill_light = LaunchConfiguration("enable_fill_light")
    detector_mode = LaunchConfiguration("detector_mode")
    color_calib_path = LaunchConfiguration("color_calib_path")
    infer_imgsz = LaunchConfiguration("infer_imgsz")
    target_timer_period = LaunchConfiguration("target_timer_period")
    enable_target_lock = LaunchConfiguration("enable_target_lock")
    lock_max_pixel_jump = LaunchConfiguration("lock_max_pixel_jump")
    color_morph_kernel_size = LaunchConfiguration("color_morph_kernel_size")
    color_erode_kernel_size = LaunchConfiguration("color_erode_kernel_size")
    color_min_area = LaunchConfiguration("color_min_area")
    color_max_area_ratio = LaunchConfiguration("color_max_area_ratio")
    color_aspect_min = LaunchConfiguration("color_aspect_min")
    color_aspect_max = LaunchConfiguration("color_aspect_max")
    color_extent_min = LaunchConfiguration("color_extent_min")
    color_solidity_min = LaunchConfiguration("color_solidity_min")
    enable_target_hold = LaunchConfiguration("enable_target_hold")
    target_hold_max_frames = LaunchConfiguration("target_hold_max_frames")
    target_hold_timeout_s = LaunchConfiguration("target_hold_timeout_s")
    target_hold_max_pixel_drift = LaunchConfiguration("target_hold_max_pixel_drift")
    target_hold_max_base_drift_mm = LaunchConfiguration("target_hold_max_base_drift_mm")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt",
        ),
        DeclareLaunchArgument(
            "handeye_path",
            default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/handeye/handeye_cam_to_eef.json",
        ),
        DeclareLaunchArgument(
            "arm_port",
            default_value="/dev/ttyUSB0",
        ),
        DeclareLaunchArgument(
            "show_window",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_fill_light",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "detector_mode",
            default_value="fusion",
        ),
        DeclareLaunchArgument(
            "color_calib_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "infer_imgsz",
            default_value="256",
        ),
        DeclareLaunchArgument(
            "target_timer_period",
            default_value="0.08",
        ),
        DeclareLaunchArgument("enable_target_lock", default_value="true"),
        DeclareLaunchArgument("lock_max_pixel_jump", default_value="160.0"),
        DeclareLaunchArgument("color_morph_kernel_size", default_value="5"),
        DeclareLaunchArgument("color_erode_kernel_size", default_value="5"),
        DeclareLaunchArgument("color_min_area", default_value="250.0"),
        DeclareLaunchArgument("color_max_area_ratio", default_value="0.35"),
        DeclareLaunchArgument("color_aspect_min", default_value="0.35"),
        DeclareLaunchArgument("color_aspect_max", default_value="3.0"),
        DeclareLaunchArgument("color_extent_min", default_value="0.25"),
        DeclareLaunchArgument("color_solidity_min", default_value="0.50"),
        DeclareLaunchArgument("enable_target_hold", default_value="true"),
        DeclareLaunchArgument("target_hold_max_frames", default_value="5"),
        DeclareLaunchArgument("target_hold_timeout_s", default_value="0.6"),
        DeclareLaunchArgument("target_hold_max_pixel_drift", default_value="80.0"),
        DeclareLaunchArgument("target_hold_max_base_drift_mm", default_value="80.0"),

        Node(
            package="red_block_grasp_ros2",
            executable="roarm_driver_node",
            name="roarm_driver_node",
            output="screen",
            parameters=[
                {
                    "port": arm_port,
                    "state_period": 0.2,
                    "auto_connect": True,
                    "enable_fill_light": enable_fill_light,
                }
            ],
        ),

        Node(
            package="red_block_grasp_ros2",
            executable="target_localizer_node",
            name="target_localizer_node",
            output="screen",
            parameters=[
                {
                    "model_path": model_path,
                    "handeye_path": handeye_path,
                    "conf_thres": 0.35,
                    "infer_imgsz": infer_imgsz,
                    "max_targets": 4,
                    "timer_period": target_timer_period,
                    "show_window": show_window,
                    "detector_mode": detector_mode,
                    "color_calib_path": color_calib_path,
                    "enable_color_detector": True,
                    "enable_yolo_detector": True,
                    "enable_target_lock": enable_target_lock,
                    "lock_max_pixel_jump": lock_max_pixel_jump,
                    "color_morph_kernel_size": color_morph_kernel_size,
                    "color_erode_kernel_size": color_erode_kernel_size,
                    "color_min_area": color_min_area,
                    "color_max_area_ratio": color_max_area_ratio,
                    "color_aspect_min": color_aspect_min,
                    "color_aspect_max": color_aspect_max,
                    "color_extent_min": color_extent_min,
                    "color_solidity_min": color_solidity_min,
                    "enable_target_hold": enable_target_hold,
                    "target_hold_max_frames": target_hold_max_frames,
                    "target_hold_timeout_s": target_hold_timeout_s,
                    "target_hold_max_pixel_drift": target_hold_max_pixel_drift,
                    "target_hold_max_base_drift_mm": target_hold_max_base_drift_mm,
                }
            ],
        ),
    ])
