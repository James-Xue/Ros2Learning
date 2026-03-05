# ros2_learning_composition

ROS 2 Component 组合深度学习包，通过**图像处理流水线**场景串联以下核心概念：

| 概念 | 如何体现 |
|------|---------|
| Intra-process 零拷贝 | Producer 使用 `unique_ptr` 发布；Processor 通过 `UniquePtr` 回调接收 |
| 动态组件加载/卸载 | `pipeline_dynamic.launch.py` 启动空容器，手动 `ros2 component load` |
| 生命周期组件 | ImageSaver 继承 `LifecycleNode`，可控制 activate/deactivate |
| 多容器架构 | `pipeline_multi_container.launch.py` 双容器对比零拷贝失效 |

## 流水线架构

```
ImageProducer ──/pipeline/raw──► ImageProcessor ──/pipeline/processed──► ImageSaver
  (Node)           64x64 RGB8       (Node)           反转像素              (LifecycleNode)
  10Hz 定时发布    unique_ptr       unique_ptr 接收                       帧数/延迟统计
```

## 构建

```bash
cd ros2_ws
colcon build --packages-select ros2_learning_composition --symlink-install
source install/setup.bash
```

## 运行

```bash
# 单容器零拷贝（推荐首先体验）
ros2 launch ros2_learning_composition pipeline_composed.launch.py

# 双容器对比（观察跨容器通信延迟增加）
ros2 launch ros2_learning_composition pipeline_multi_container.launch.py

# 动态加载演示
ros2 launch ros2_learning_composition pipeline_dynamic.launch.py
# 另一终端手动加载：
ros2 component load /pipeline_container ros2_learning_composition \
    ros2_learning_composition::ImageProducer -e use_intra_process_comms:=true
ros2 component load /pipeline_container ros2_learning_composition \
    ros2_learning_composition::ImageProcessor -e use_intra_process_comms:=true
ros2 component load /pipeline_container ros2_learning_composition \
    ros2_learning_composition::ImageSaver -e use_intra_process_comms:=true
```

### 激活 ImageSaver 生命周期

ImageSaver 是 LifecycleNode，需要手动激活才会开始统计：

```bash
ros2 lifecycle set /image_saver configure
ros2 lifecycle set /image_saver activate
# 查看统计后停止
ros2 lifecycle set /image_saver deactivate
```

### 开启 DEBUG 日志查看指针地址

```bash
ros2 launch ros2_learning_composition pipeline_composed.launch.py \
    --ros-args --log-level debug
```

## 测试

```bash
colcon test --packages-select ros2_learning_composition
colcon test-result --verbose
```

8 个 GTest 用例覆盖：组件实例化、消息发布/接收、像素变换验证、生命周期转换、帧统计、端到端流水线、非 Active 状态忽略。
