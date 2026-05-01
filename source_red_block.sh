#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/sunrise/dog/ros2_red_block_ws/install/local_setup.bash

export AMENT_PREFIX_PATH=/home/sunrise/dog/ros2_red_block_ws/install/red_block_grasp_ros2:$AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH=/home/sunrise/dog/ros2_red_block_ws/install/red_block_grasp_ros2:$CMAKE_PREFIX_PATH
export PATH=/home/sunrise/dog/ros2_red_block_ws/install/red_block_grasp_ros2/lib/red_block_grasp_ros2:$PATH
