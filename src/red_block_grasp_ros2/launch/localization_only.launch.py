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
                }
            ],
        ),
    ])
