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
                    "max_targets": 4,
                    "timer_period": 0.3,
                    "show_window": show_window,
                }
            ],
        ),
    ])
