# dog_visual_grasp_bridge

这个目录用于桥接 dog-arm 视觉定位结果和官方 RoArm-M3 MoveIt2 控制接口。

它不替代原有视觉包。当前主流程是：

`red_block_grasp_ros2/localization_only.launch.py`
-> `/red_block/target_base`
-> `visual_moveit_grasp.py`
-> `/move_line_cmd`
-> `/gripper_cmd`

## 作用

- 原有 `red_block_grasp_ros2` 继续负责：
  - Orbbec 相机
  - 红色物块识别
  - 深度定位
  - 手眼转换
  - 发布 `/red_block/target_base`
- 本目录只负责：
  - 订阅 `/red_block/target_base`
  - 冻结稳定目标
  - 调用官方 `/move_line_cmd`
  - 发布 `/gripper_cmd`

## 不要启动

- dog-arm 的 `roarm_driver_node`
- dog-arm 的 `open_loop_grasp_task_node`
- `open_loop_grasp.launch.py`

## 必须启动官方 roarm_ws

终端 1：官方 driver

```bash
cd /home/sunrise/dog/roarm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROARM_MODEL=roarm_m3
sudo chmod 666 /dev/ttyUSB0
ros2 run roarm_driver roarm_driver --ros-args -r serial_port:=/dev/ttyUSB0
```

终端 2：官方 command_control

```bash
cd /home/sunrise/dog/roarm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROARM_MODEL=roarm_m3
ros2 launch roarm_moveit_cmd command_control.launch.py
```

终端 3：dog-arm 视觉定位

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 launch red_block_grasp_ros2 localization_only.launch.py \
  show_window:=true \
  detector_mode:=color \
  color_calib_path:=/tmp/red_color_calib_tuned.yaml \
  enable_target_lock:=true \
  enable_target_hold:=true
```

终端 4：桥接抓取

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source /home/sunrise/dog/ros2_red_block_ws/install/setup.bash
python3 /home/sunrise/dog/ros2_red_block_ws/dog_visual_grasp_bridge/visual_moveit_grasp.py
```

## 测试前检查

```bash
ros2 service list | grep cmd
ros2 topic info /gripper_cmd
ros2 topic echo /red_block/target_base
ros2 topic echo /red_block/base_adjust_request
```

## 夹爪实测值

- `gripper_open_value=1.5` 为张开
- `gripper_close_value=0.7` 适合当前调试闭合
- `gripper_close_value_strong=0.5` 可以夹得更紧，但不要一开始默认使用

## 注意事项

- 不要同时启动两个机械臂 driver
- 不要让 dog-arm 的 `roarm_driver_node` 和官方 `roarm_driver` 同时占用 `/dev/ttyUSB0`
- 当前第一版只支持单个红块抓取
- 两个红块连续抓取后续再做
- 箱子 ROI / slot 后续再做
