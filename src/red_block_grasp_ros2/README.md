# red_block_grasp_ros2

基于 ROS2 Humble 的红色物料识别与 RoArm-M3 机械臂抓取项目。

## 项目目标

本项目用于完成红色物料抓取任务：

- 使用 Orbbec RGBD 相机获取图像和深度
- 使用 YOLO 识别红色物料
- 结合深度图计算目标三维坐标
- 通过手眼标定转换到机械臂 base 坐标系
- 控制 RoArm-M3 移动到红色物料正上方
- 后续继续完成下降、夹取、抬升和放置

## ROS2 节点结构

### roarm_driver_node

机械臂驱动节点，唯一占用 `/dev/ttyUSB0`。

发布：

- `/roarm_m3/state`

订阅：

- `/roarm_m3/cmd`

### target_localizer_node

视觉定位节点，负责相机、YOLO、深度定位和手眼转换。

发布：

- `/red_block/target_base`

订阅：

- `/roarm_m3/state`

### visual_servo_task_node

视觉闭环任务节点，负责根据目标位置小步控制机械臂运动。

发布：

- `/roarm_m3/cmd`
- `/red_block/visual_servo_state`

订阅：

- `/roarm_m3/state`
- `/red_block/target_base`

## 编译

    cd /home/sunrise/dog/ros2_red_block_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select red_block_grasp_ros2 --event-handlers console_direct+

## 运行

    cd /home/sunrise/dog/ros2_red_block_ws
    source source_red_block.sh
    ros2 launch red_block_grasp_ros2 visual_servo_task.launch.py

注意：运行该 launch 时不要同时运行 `yolo_camera_node`，否则会抢占 Orbbec 相机。

## 常用调试命令

查看目标定位：

    ros2 topic echo /red_block/target_base

查看视觉闭环状态：

    ros2 topic echo /red_block/visual_servo_state

查看机械臂状态：

    ros2 topic echo /roarm_m3/state

LED 测试：

    ros2 topic pub --once /roarm_m3/cmd std_msgs/msg/String "{data: '{\"type\":\"set_led\",\"led\":120}'}"

回初始姿态：

    ros2 topic pub --once /roarm_m3/cmd std_msgs/msg/String "{data: '{\"type\":\"set_initial_pose\",\"targets_deg\":{\"b\":0.0,\"s\":0.0,\"e\":70.0,\"t\":90.0,\"r\":-90.0,\"g\":null},\"speed\":35.0,\"acc\":35.0}'}"

## 困难样本采集

困难样本保存目录：

    /home/sunrise/dog/ros2_red_block_ws/hard_samples

查看样本：

    ls -lh /home/sunrise/dog/ros2_red_block_ws/hard_samples | tail

这些样本后续用于补充训练 YOLO v2。

## 当前状态

已完成：

- ROS2 功能包搭建
- 机械臂驱动节点
- YOLO 视觉定位节点
- 视觉闭环任务节点
- launch 一键启动
- 困难样本采集逻辑

待完成：

- 修复并验证困难样本稳定保存
- 采集并标注困难样本
- 重新训练 YOLO v2
- 接入下降、夹取、抬升和放置流程
- 后续考虑 RDK X5 BPU 推理加速
