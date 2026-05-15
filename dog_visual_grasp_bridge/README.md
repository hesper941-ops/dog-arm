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

## snapshot 过期策略

- `stage_target_max_age_s` 只用于开始抓取前的目标新鲜度检查。
- `allow_snapshot_expire_during_motion` 默认值为 `true`。
- 抓取流程一旦开始，桥接节点会使用已经冻结的 `snapshot` 完成当前单次抓取。
- 这样可以避免机械臂运动过程中因为视觉短时丢目标或 `snapshot` 超时而中途进入 `RECOVER`。

## 观察关节姿态

- 旧的全局观察位不是 `xyz` 点，而是关节角姿态：`B=0`、`S=0`、`E=70`、`T=90`、`R=-90`。
- 这个观察姿态是相机能看见箱子全局的前提。
- `/move_line_cmd` 只用于后续 `pre_grasp / grasp / lift`，不用于观察位。

根据官方文档，当前已确认：

- `roarm_description display.launch.py` 的关节滑块会经由 `/joint_states` 控制真机。
- 官方关节控制 topic 是 `/joint_states`。
- 消息类型是 `sensor_msgs/msg/JointState`。
- 关节角单位是 `rad`。
- 官方 `roarm_driver` 处理 RoArm-M3 时，`JointState` 必须包含正确的 `name`，不能只发 `position`。
- RoArm-M3 需要这 6 个关节名：
  - `base_link_to_link1`
  - `link1_to_link2`
  - `link2_to_link3`
  - `link3_to_link4`
  - `link4_to_link5`
  - `link5_to_gripper_link`
- 旧观察姿态 `B/S/E/T/R` 分别映射到前 5 个关节，第 6 个 `link5_to_gripper_link` 使用 `observe_gripper_rad`，默认 `1.5`。

桥接脚本现在支持在启动后先发布一次官方观察关节姿态，再等待 `observe_wait_s` 后进入 `WAIT_TARGET`。

如果你在 X5 端已经确认了官方 URDF 的精确 joint names，可以直接在 `config/grasp_config.yaml` 里覆盖 `official_joint_names`。当前默认留空，脚本只发布 `position` 数组，不去猜测官方源码里的精确 joint name 字符串。
