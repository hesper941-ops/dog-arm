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
    enable_pick_place_sequence = LaunchConfiguration("enable_pick_place_sequence")
    servo_min_z_mm = LaunchConfiguration("servo_min_z_mm")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n_v3_mix.pt",
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
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_pick_place_sequence",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "servo_min_z_mm",
            default_value="110.0",
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
                    "conf_thres": 0.25,
                    "infer_imgsz": 320,
                    "max_targets": 4,
                    "timer_period": 0.3,
                    "show_window": show_window,

                    "safe_roi_x_min_ratio": 0.12,
                    "safe_roi_x_max_ratio": 0.88,
                    "safe_roi_y_min_ratio": 0.12,
                    "safe_roi_y_max_ratio": 0.88,
                    "enable_target_lock": True,
                    "lock_max_pixel_jump": 160.0,
                    "center_weight": 0.25,
                    "base_filter_alpha": 0.55,

                    "save_hard_samples": False,
                    "hard_sample_conf_thres": 0.75,
                    "hard_sample_interval_s": 0.8,
                    "force_save_samples": False,
                    "force_sample_interval_s": 1.5,
                    "hard_sample_dir": "/home/sunrise/dog/ros2_red_block_ws/hard_samples",

                }
            ],
        ),

        Node(
            package="red_block_grasp_ros2",
            executable="visual_servo_task_node",
            name="visual_servo_task_node",
            output="screen",
            parameters=[
                {
                    "auto_start": True,
                    "move_once_only": True,

                    "init_b_deg": 0.0,
                    "init_s_deg": 0.0,
                    "init_e_deg": 70.0,
                    "init_t_deg": 90.0,
                    "init_r_deg": -90.0,
                    "init_speed_deg_s": 35.0,
                    "init_acc": 35.0,
                    "initial_wait_s": 3.0,

                    "enable_b_scan": True,
                    "b_scan_offsets": "0,-8,8,-16,16,-24,24",
                    "scan_timeout_s": 8.0,
                    "after_scan_pose_wait_s": 2.5,

                    "target_timeout_s": 1.0,
                    "max_step_mm": 25.0,
                    "edge_step_mm": 12.0,
                    "move_speed": 0.10,
                    "step_wait_s": 2.0,

                    "enable_e_pixel_servo": False,

                    "target_xy_tolerance_mm": 25.0,
                    "target_z_tolerance_mm": 25.0,

                    "safe_above_offset_mm": 120.0,
                    "grasp_offset_x_mm": 0.0,
                    "grasp_offset_y_mm": 0.0,
                    "grasp_offset_z_mm": 0.0,
                    "min_safe_z_mm": 30.0,
                    "servo_min_z_mm": servo_min_z_mm,
                    "servo_recover_z_margin_mm": 30.0,
                    "servo_recover_max_attempts": 3,

                    "image_width": 640,
                    "image_height": 480,
                    "safe_roi_x_min_ratio": 0.20,
                    "safe_roi_x_max_ratio": 0.80,
                    "safe_roi_y_min_ratio": 0.20,
                    "safe_roi_y_max_ratio": 0.80,

                    "base_x_min": 80.0,
                    "base_x_max": 700.0,
                    "base_y_min": -450.0,
                    "base_y_max": 450.0,
                    "base_z_min": -30.0,
                    "base_z_max": 380.0,

                    "descend_test_mm": 30.0,
                    "descend_step_mm": 5.0,
                    "descend_control_mode": "pixel",
                    "descend_lock_xy": True,
                    "descend_xy_step_mm": 0.0,
                    "descend_x_comp_mm_per_mm": 0.0,
                    "descend_y_comp_mm_per_mm": 0.0,
                    "descend_desired_pixel_u": 320.0,
                    "descend_desired_pixel_v": 240.0,
                    "descend_pixel_deadband": 35.0,
                    "descend_pixel_kp_mm_per_px": 0.04,
                    "descend_pixel_max_xy_step_mm": 6.0,
                    "descend_pixel_x_sign": 1.0,
                    "descend_pixel_y_sign": 1.0,
                    "descend_min_z_mm": 30.0,
                    "descend_min_confidence": 0.60,
                    "descend_speed": 0.06,
                    "descend_wait_s": 2.0,
                    "descend_step_wait_s": 1.0,

                    "enable_pick_place_sequence": enable_pick_place_sequence,
                    "gripper_close_deg": 55.0,
                    "gripper_open_deg": 110.0,
                    "gripper_speed_deg_s": 25.0,
                    "gripper_acc": 25.0,
                    "gripper_wait_s": 1.0,
                    "lift_up_mm": 80.0,
                    "lift_speed": 0.08,
                    "lift_wait_s": 2.0,
                    "place_x_mm": 260.0,
                    "place_y_mm": 180.0,
                    "place_z_mm": 120.0,
                    "place_speed": 0.10,
                    "place_wait_s": 2.0,
                }
            ],
        ),
    ])
