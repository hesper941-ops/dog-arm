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
