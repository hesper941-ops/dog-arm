# dog_visual_grasp_bridge

这个目录负责把视觉结果桥接到官方 `RoArm-M3` 的控制接口：

`/red_block/target_base` -> `visual_moveit_grasp.py` -> `/move_line_cmd` + `/gripper_cmd`

它不替代视觉包，也不修改 handeye、相机内参、YOLO 或官方机械臂参数。

## Runtime fix

2026-05 的运行时修复已经包含在本目录：

- `visual_moveit_grasp.py` 使用 `ReentrantCallbackGroup`
- main loop 使用 `MultiThreadedExecutor(num_threads=2)`
- 新增 `move_line_timeout_s`，默认 `120.0`
- 新增 `service_wait_timeout_s`，默认 `5.0`
- `DONE` 和 `RECOVER` 会提前返回，不再持续触发 snapshot 过期检查

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source /home/sunrise/dog/ros2_red_block_ws/install/setup.bash
python3 /home/sunrise/dog/ros2_red_block_ws/dog_visual_grasp_bridge/visual_moveit_grasp.py
```

## Config

配置文件在 `config/grasp_config.yaml`。

- `move_line_timeout_s`: `/move_line_cmd` 单次调用超时
- `service_wait_timeout_s`: 等待服务可用的超时
- `allow_snapshot_expire_during_motion`: 运动阶段是否允许 snapshot 超龄但继续执行

## Snapshot policy

- `IDLE` 不检查 snapshot 过期
- `WAIT_TARGET` 只检查开始前 snapshot 是否仍然新鲜，不刷 warning
- `OPEN_GRIPPER`
- `MOVE_TO_PRE_GRASP`
- `MOVE_LINE_TO_GRASP`
- `CLOSE_GRIPPER`
- `LIFT`

只有上面这些运动阶段会检查 snapshot 过期。

- `RECOVER` 进入时会清理 snapshot，然后保持该状态，不再刷屏
- `DONE` 进入时也会清理 snapshot，并且不会因为 snapshot 缺失再转回 `RECOVER`
