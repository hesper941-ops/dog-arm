# red_block_grasp_ros2

基于 ROS2 Humble 的红色物料识别与 RoArm-M3 机械臂抓取项目。

## 项目目标

本项目用于完成红色物料抓取任务：

- 使用 Orbbec RGBD 相机获取图像和深度
- 使用 YOLO 识别红色物料
- 结合深度图计算目标三维坐标
- 通过手眼标定转换到机械臂 base 坐标系
- 控制 RoArm-M3 移动到红色物料正上方
- 执行下降测试验证下降方向和安全高度
- 可选执行闭爪、抬升和放置流程

## 项目状态（2026-05-08）

### 已完成
- ✅ 新版 YOLO 模型（red_block_yolo11n_v3_mix.pt）已训练并验证通过
- ✅ 识别红块功能正常
- ✅ 计算 base 坐标功能正常
- ✅ 视觉闭环小步移动功能正常
- ✅ 移动到预抓取点（距红块上方 120mm）功能正常
- ✅ 下降测试状态已接入
- ✅ 闭爪、抬升、放置状态已接入，可通过参数开关启用

### 进行中
- 🔄 下降测试功能验证
- 🔄 闭爪角度、抬升距离、放置点现场调优

### 待接入
- ⏳ 实物抓取闭环验证

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

视觉闭环任务节点，负责根据目标位置小步控制机械臂运动，并执行下降测试。

#### 状态机流程

```
INIT
  ↓
WAIT_INITIAL (等待初始姿态到达)
  ↓
WAIT_TARGET (等待识别到红块)
  ↓
SERVO_STEP (视觉闭环小步移动)
  ↓
WAIT_AFTER_STEP (等待移动完成)
  ↓
REACHED_PRE_GRASP (到达预抓取点，准备下降)
  ↓
DESCEND_TEST (执行下降测试，沿 z 方向下降)
  ↓
WAIT_AFTER_DESCEND (等待下降完成)
  ↓
CLOSE_GRIPPER (可选，闭爪)
  ↓
WAIT_AFTER_CLOSE_GRIPPER (可选，等待闭爪完成)
  ↓
LIFT_AFTER_GRASP (可选，抬升)
  ↓
WAIT_AFTER_LIFT (可选，等待抬升完成)
  ↓
MOVE_TO_PLACE (可选，移动到放置点)
  ↓
WAIT_AFTER_PLACE (可选，等待到达放置点)
  ↓
OPEN_GRIPPER (可选，开爪释放)
  ↓
WAIT_AFTER_OPEN_GRIPPER (可选，等待开爪完成)
  ↓
DONE (完成，发布 reached_descend_test 或 picked_and_placed)
```

#### 下降测试参数

- `descend_test_mm`: 下降距离，默认 30.0 mm
- `descend_step_mm`: 单次下降步长，默认 5.0 mm
- `descend_xy_step_mm`: 下降时单步 XY 修正上限，默认 8.0 mm
- `descend_min_z_mm`: 最低安全高度，默认 30.0 mm
- `descend_min_confidence`: 下降阶段最低目标置信度，默认 0.60
- `descend_speed`: 下降速度，默认 0.06（比视觉闭环移动更慢）
- `descend_wait_s`: 下降后等待时间，默认 2.0 s
- `descend_step_wait_s`: 每个下降小步后的等待时间，默认 1.0 s

#### 抓取、抬升、放置参数

- `enable_pick_place_sequence`: 是否在下降后继续执行抓取放置流程，默认 false
- `gripper_close_deg`: 闭爪角度，默认 55.0 deg
- `gripper_open_deg`: 开爪角度，默认 110.0 deg
- `gripper_speed_deg_s`: 夹爪速度，默认 25.0 deg/s
- `gripper_acc`: 夹爪加速度，默认 25.0
- `gripper_wait_s`: 闭爪或开爪后的等待时间，默认 1.0 s
- `lift_up_mm`: 闭爪后抬升距离，默认 80.0 mm
- `lift_speed`: 抬升速度，默认 0.08
- `lift_wait_s`: 抬升后等待时间，默认 2.0 s
- `place_x_mm`, `place_y_mm`, `place_z_mm`: 放置点 base 坐标，默认 260.0, 180.0, 120.0 mm
- `place_speed`: 移动到放置点速度，默认 0.10
- `place_wait_s`: 到达放置点后等待时间，默认 2.0 s

#### 安全保护

- 视觉闭环阶段目标 z 不低于 `servo_min_z_mm`
- 如果当前 z 已低于 `servo_min_z_mm`，先原地抬升回安全高度
- 红块在图像边缘时，不允许继续根据不稳定深度向下移动
- 下降阶段改为视觉守护小步下降，每步检查目标新鲜度、置信度和像素安全区
- 下降前检查目标 z 坐标是否低于 `descend_min_z_mm`
- 抬升和放置前继续调用 `check_target_range` 检查工作空间
- 若超出安全范围，状态机进入 FAIL，发布错误信息，不继续执行

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

启用下降后的抓取、抬升、放置流程：

    ros2 launch red_block_grasp_ros2 visual_servo_task.launch.py enable_pick_place_sequence:=true

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

## 开发记录：训练集与困难样本位置确认

当前已经在 RDK X5 上确认红色物料训练相关数据位置。

### 困难样本

困难样本保存目录：

/home/sunrise/dog/ros2_red_block_ws/hard_samples

该目录中包含自动保存的困难样本图片和对应 JSON 信息，文件名类型包括 force_conf、lowconf、no_detection、force_no_detection。

这些样本主要用于补充新版 YOLO 训练集，重点解决低置信度、漏检、边缘视角和复杂背景下的识别问题。

### ROS2 工作区内的数据集

当前 ROS2 工作区内已经存在两个新版数据集目录：

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2_full

其中 red_block_v2_full 更可能是后续优先使用的完整训练集版本，需要进一步检查其中的图片数量、标签数量和 red_block_v2_full.yaml 配置是否正确。

### 旧工程数据集与旧训练结果

旧工程中仍保留了早期红块训练数据和训练结果：

/home/sunrise/dog/red_block_grasp/dataset/red_block_dataset

/home/sunrise/dog/red_block_grasp/runs/detect/red_block_yolo11n/weights/best.pt

旧训练结果可以作为对照模型，但后续 ROS2 工程应优先使用重新整理后的 red_block_v2_full 训练新版 YOLO。

### 当前 ROS2 工程正在使用的模型

当前 ROS2 工程默认加载的模型为：

/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt

后续新版 YOLO 训练完成后，需要将新版 best.pt 复制到 models 目录，或修改 launch 文件中的 model_path 指向新版模型。

### 下一步任务

接下来应先检查并整理数据集：

1. 统计 red_block_v2 和 red_block_v2_full 的图片数量与标签数量。
2. 检查 red_block_v2_full.yaml 中的 train、val、names 是否正确。
3. 抽查标签是否与图片一一对应。
4. 确认无问题后训练新版 YOLO。
5. 训练完成后替换 ROS2 工程中的模型并进行现场识别验证。

## 开发记录：新版 YOLO 数据集检查

当前已在 RDK X5 上完成新版红色物料数据集位置与基本完整性检查。

已确认的数据集配置文件：

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2/red_block_v2.yaml

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2_full/red_block_v2_full.yaml

两个 yaml 文件中的 path、train、val 和类别 names 均存在，类别为 red_block。

数据量统计：

red_block_v2:
- train 图片 36 张，train 标签 36 个
- val 图片 24 张，val 标签 24 个
- train 中存在少量空标签文件

red_block_v2_full:
- train 图片 99 张，train 标签 99 个
- val 图片 24 张，val 标签 24 个
- train 中存在较多空标签文件

当前判断：

red_block_v2_full 数据量更多，更适合作为新版 YOLO 的候选训练集。但是由于其中存在较多空标签文件，下一步需要确认这些空标签图片到底是无红块负样本，还是漏标样本。

如果空标签图片确实是不含红块的负样本，可以保留用于降低误检。如果空标签图片中实际存在红块，则需要补标后再训练。

下一步任务：

1. 统计 red_block_v2_full 中空标签文件总数。
2. 抽查空标签对应图片。
3. 判断空标签是负样本还是漏标。
4. 修正数据集后再训练新版 YOLO。

## 开发记录：red_block_v2_full 手动标注检查完成

当前已经完成 red_block_v2_full 数据集的人工标注检查与修改。

数据集路径：

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2_full

本次使用自定义 YOLO 标注脚本进行检查，重点检查了：

- 空标签样本
- lowconf 低置信度样本
- no_detection 漏检样本
- force_no_detection 强制采样但未识别样本
- force_conf_0.2 到 force_conf_0.5 的低置信度样本

标注原则：

- 图片中有红块但无框：补充 red_block 标注框
- 图片中已有框但位置不准：手动修正
- 图片中确实没有红块：保留为空标签，作为负样本
- 图片模糊但仍能判断红块位置：保留并标注
- 明显不可用图片：后续合并训练集前再统一筛查

已添加自定义标注入口：

/home/sunrise/dog/ros2_red_block_ws/tools/manual_yolo_labeler.py

快捷命令：

biaozhu

常用方式：

biaozhu empty
biaozhu hard
biaozhu train
biaozhu val

当前状态：

red_block_v2_full 已完成手动标注检查，可以进入下一步：合并旧数据集与新困难样本数据集，构建 red_block_v3_mix。

下一步任务：

1. 检查旧数据集 red_block_dataset 的图片和标签完整性。
2. 新建混合数据集 red_block_v3_mix。
3. 合并旧数据集和 red_block_v2_full。
4. 使用旧模型 best.pt 作为初始权重继续训练新版 YOLO。
5. 训练完成后替换 ROS2 工程中的 red_block_yolo11n.pt 并进行现场识别验证。

## 开发记录：red_block_v3_mix 混合数据集标注检查完成

当前已经完成 red_block_v3_mix 混合数据集的人工标注检查与修改。

混合数据集路径：

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v3_mix

该数据集由两部分组成：

1. 旧工程红块数据集：

/home/sunrise/dog/red_block_grasp/dataset/red_block_dataset

2. 新采集困难样本数据集：

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2_full

混合数据集配置文件：

/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v3_mix/red_block_v3_mix.yaml

当前已经使用自定义标注脚本对混合数据集进行了人工检查，重点处理：

- 错误标注框
- 偏移标注框
- 漏标红块
- 空标签样本
- lowconf、no_detection、force_conf、force_no_detection 等困难样本

标注工具入口：

/home/sunrise/dog/ros2_red_block_ws/tools/manual_yolo_labeler.py

混合数据集快捷打开命令：

openmix

常用方式：

openmix train
openmix val
openmix empty-train
openmix empty-val
openmix hard

重要说明：

从当前节点开始，red_block_v3_mix 作为正式训练集使用。不要再执行 build_red_block_v3_mix.py --overwrite，否则会覆盖已经人工修改过的混合数据集标注。

下一步任务：

1. 使用旧模型 best.pt 作为初始权重继续训练新版 YOLO。
2. 训练输出新版 red_block_v3_mix 模型。
3. 将新版 best.pt 替换或复制到 ROS2 工程 models 目录。
4. 修改 launch 或模型路径，使用新版 YOLO。
5. 现场验证红块识别稳定性。

## 开发记录：新版 YOLO 模型现场验证通过

当前已经将 PC 端训练得到的新版 YOLO 模型导入 RDK X5，并在 ROS2 主流程中完成现场验证。

新版模型路径：

/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n_v3_mix.pt

当前 launch 已使用新版模型作为默认模型。

验证结果：

- target_localizer_node 能正常加载新版 YOLO 模型。
- Orbbec RGBD 相机能正常启动。
- /red_block/target_base 能发布有效目标坐标。
- visual_servo_task_node 能进入 SERVO_STEP。
- 机械臂驱动节点能收到 move_pose 命令。
- 当前流程已经能够完成视觉闭环移动到红块上方。
- 任务状态能够到达 DONE，即 reached_pre_grasp。

当前已完成的主流程：

识别红块
→ 计算目标 base 坐标
→ 视觉闭环小步移动
→ 到达红块上方预抓取点

下一步任务：

1. 在稳定识别基础上接入下降动作。
2. 接入闭爪动作。
3. 接入抬升动作。
4. 最后再接入放置动作。
5. 保持分阶段动作，不直接一次性冲到最终抓取点。

