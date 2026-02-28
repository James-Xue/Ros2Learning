# ros2_learning_yolo_detector

> 基于 YOLOv8 的 ROS 2 目标检测节点（Python），订阅相机图像并发布标准 `vision_msgs` 检测结果与标注图像。

## 功能描述

本包实现了一个 Python 目标检测节点（`yolo_detector`），将 Ultralytics YOLOv8 模型集成到 ROS 2 数据流中。节点订阅任意相机图像话题，对每一帧图像调用 YOLO 推理，将检测到的每个目标封装为 `vision_msgs/msg/Detection2D`（含边界框中心坐标、尺寸及类别置信度），整体以 `Detection2DArray` 发布；同时使用 YOLO 内置绘图函数生成标注图像并以 `sensor_msgs/Image` 发布，可直接用 RViz2 或 `rqt_image_view` 实时预览。

本节点演示了将深度学习推理引入 ROS 2 节点的典型模式：通过 `cv_bridge` 在 ROS `Image` 消息与 OpenCV `BGR` 格式之间转换；延迟导入 `ultralytics` 以避免包未安装时节点初始化阶段即崩溃，而是在首次实际调用时给出明确错误提示。

节点采用 `ament_python` 构建类型，所有配置通过 ROS 2 参数系统暴露，支持在 launch 文件中覆盖模型名称、置信度阈值和图像话题，无需修改源码即可切换不同 YOLOv8 变体（n/s/m/l/x）或对接不同相机节点。

## 运行命令

```bash
# 0. 安装 Python 依赖（仅需一次）
pip install ultralytics

# 1. source 环境
source /opt/ros/jazzy/setup.bash
source /root/Ros2Learning/ros2_ws/install/setup.bash

# 2. 使用 launch 文件启动（推荐）
ros2 launch ros2_learning_yolo_detector yolo_detector.launch.py

# 3. 使用更大模型并降低置信度阈值
ros2 launch ros2_learning_yolo_detector yolo_detector.launch.py \
  model_name:=yolov8s.pt \
  confidence_threshold:=0.4

# 4. 对接自定义相机话题
ros2 launch ros2_learning_yolo_detector yolo_detector.launch.py \
  image_topic:=/camera/color/image_raw

# 5. 直接用 ros2 run 启动（覆盖参数示例）
ros2 run ros2_learning_yolo_detector yolo_detector \
  --ros-args \
    -p model_name:=yolov8n.pt \
    -p confidence_threshold:=0.5 \
    -p image_topic:=/camera/image_raw

# 6. 查看检测结果（结构化数据）
ros2 topic echo /detections

# 7. 在 RViz2 中预览标注图像
# 添加 Image 显示，话题选择 /detection_image

# 8. 用 rqt_image_view 查看标注图像
ros2 run rqt_image_view rqt_image_view /detection_image
```

> 首次运行时 Ultralytics 会自动从网络下载所指定的模型权重文件（如 `yolov8n.pt`，约 6 MB），需要网络连接。下载完成后缓存在 `~/.ultralytics/` 目录，后续运行无需重复下载。

## 输入/输出

### 订阅的话题（Subscriber）

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/camera/image_raw`（可通过参数 `image_topic` 修改） | `sensor_msgs/msg/Image` | 输入原始相机图像，编码格式须为 `bgr8` 或 `rgb8` 等 `cv_bridge` 支持的格式 |

### 发布的话题（Publisher）

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/detections` | `vision_msgs/msg/Detection2DArray` | 当前帧所有检测结果；`header` 与输入图像同步 |
| `/detection_image` | `sensor_msgs/msg/Image` | 叠加了边界框和类别标签的标注图像，编码 `bgr8` |

#### `Detection2DArray` 字段说明

每个 `Detection2D` 包含：
- `bbox.center.position.x/y`：边界框中心像素坐标
- `bbox.size_x/size_y`：边界框宽度和高度（像素）
- `results[0].hypothesis.class_id`：类别名称字符串（如 `person`、`car`）
- `results[0].hypothesis.score`：该检测框的置信度（0.0~1.0）

### 可配置的节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `model_name` | string | `yolov8n.pt` | YOLO 模型名称；可填 `yolov8n/s/m/l/x.pt`，也可填本地路径 |
| `confidence_threshold` | double | `0.5` | 检测置信度阈值，低于此值的检测框被过滤 |
| `image_topic` | string | `/camera/image_raw` | 订阅的输入图像话题 |

### 可选的 launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model_name` | `yolov8n.pt` | 传入节点的 `model_name` 参数 |
| `confidence_threshold` | `0.5` | 传入节点的 `confidence_threshold` 参数 |
| `image_topic` | `/camera/image_raw` | 传入节点的 `image_topic` 参数 |

## 验收测试

本包的 `package.xml` 声明了 `ament_copyright`、`ament_flake8`、`ament_pep257` 和 `python3-pytest` 测试依赖，但当前未实现任何自定义测试用例，代码风格检查亦未配置。

```bash
# 运行（目前无实际测试用例，ament 检查工具可能报告 0 项通过）
cd /root/Ros2Learning/ros2_ws
colcon test --packages-select ros2_learning_yolo_detector
colcon test-result --verbose
```

预期通过数量：**0**（暂无，待补充）

后续可添加的测试方向：
- 使用 `pytest` + `unittest.mock` 对 `image_callback` 进行单元测试，注入模拟检测结果，验证 `Detection2DArray` 的字段填充逻辑
- 验证置信度过滤：确认低于阈值的检测框不出现在发布消息中
- 验证 `cv_bridge` 异常时节点不崩溃而是打印错误并继续运行
- 配置 `ament_flake8` / `ament_pep257` 进行代码风格自动检查

## 已知限制

- **运行时强依赖 `ultralytics`**：该库不在 ROS 2 的标准依赖范围内，必须通过 `pip install ultralytics` 单独安装，`colcon build` 和 `rosdep` 均不会自动处理此依赖。
- **首次启动需要网络**：指定的模型权重文件（如 `yolov8n.pt`）若未本地缓存，Ultralytics 会在节点构造函数中触发下载，可能导致启动延迟或在无网络环境下直接失败。
- **无相机节点**：本包不提供相机驱动，需要用户自行启动能发布 `sensor_msgs/Image` 的相机节点（如 `usb_cam`、`realsense2_camera`、Gazebo 相机插件等）。
- **处理速度受硬件限制**：默认使用 CPU 推理。在无 GPU 的环境（如 WSL2）下，`yolov8n.pt` 处理单帧约需 100~500 ms，无法实时处理高帧率图像流。如需提速，可通过 `model_name:=yolov8n.pt` 指定最小模型，或在宿主机上配置 CUDA 环境后安装支持 GPU 的 `torch`。
- **仅支持单张图像推理**：`image_callback` 每次只处理单帧，`results[0]` 取第一个结果，不支持批量推理优化。
- **没有话题重映射配置**：当前实现通过节点参数 `image_topic` 切换输入话题，不使用 ROS 2 的话题重映射机制（`--ros-args --remap`），两种方式均可工作，但参数方式优先。
