# 🧵 ROS 2 Multithreading & Executor Demo

## 📋 概述

这个包演示了 ROS 2 **执行器（Executor）** 和 **回调组（CallbackGroup）** 的核心概念，通过对比单线程和多线程执行器处理阻塞任务的不同表现，帮助理解 ROS 2 的并发机制。

## 🎯 核心知识点

### 1️⃣ Executor（执行器）

| Executor 类型 | 线程数 | 并发能力 | 适用场景 |
|---|---|---|---|
| **SingleThreadedExecutor** | 1 | ❌ 无并发 | 简单节点、调试 |
| **MultiThreadedExecutor** | 多个 | ✅ 支持并发 | 复杂系统、实时性要求高 |

### 2️⃣ CallbackGroup（回调组）

| 类型 | 同组回调的并发性 | 示例场景 |
|---|---|---|
| **MutuallyExclusive**<br/>（互斥组） | ❌ 同一时刻只能执行一个 | 访问共享状态的回调 |
| **Reentrant**<br/>（可重入组） | ✅ 可以并行执行 | 独立的耗时任务 |

> [!IMPORTANT]
> **关键规则**：即使使用 `MultiThreadedExecutor`，**同一个 MutuallyExclusive 组内的回调仍然会串行执行**。要实现真正的并发，必须将回调分配到**不同的互斥组**或**可重入组**。

---

## 🏗️ 包结构

```
ros2_learning_multithreading/
├── include/ros2_learning_multithreading/
│   └── blocking_node.hpp          # 演示节点头文件
├── src/
│   ├── blocking_node.cpp          # 节点实现（两个定时器演示）
│   └── executor_demo_main.cpp     # 主程序（支持切换执行器类型）
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🧪 演示场景

### 节点组成

`BlockingNode` 包含两个定时器：

| 定时器 | 周期 | 功能 | 回调组 | 模拟场景 |
|---|---|---|---|---|
| **心跳定时器** | 500ms | 快速打印心跳 | `callback_group_1_` | 底盘控制指令 |
| **繁重计算定时器** | 3s | **阻塞 2 秒** | `callback_group_2_` | 全局路径规划 |

---

## 🚀 使用方法

### 编译
```bash
cd ~/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_multithreading
source install/setup.bash
```

### 运行对比实验

#### 实验 A：单线程执行器（观察阻塞现象）
```bash
ros2 run ros2_learning_multithreading executor_demo
```

**预期现象**：
```
[INFO] Executor Type: single
[INFO] Using SingleThreadedExecutor (Blocking!)
[INFO] [心跳] 咚! 线程 ID: 140123456789
[WARN] [计算] 开始复杂计算... 线程 ID: 140123456789 (预计阻塞 2秒)
# ⚠️ 心跳暂停 2 秒！（被计算阻塞）
[WARN] [计算] 计算完成！
[INFO] [心跳] 咚! 线程 ID: 140123456789
```

**分析**：
- ❌ 繁重计算阻塞了心跳定时器
- ❌ 所有回调运行在**同一线程**
- ❌ 不适合有实时性要求的系统

---

#### 实验 B：多线程执行器（心跳不受影响）
```bash
ros2 run ros2_learning_multithreading executor_demo --ros-args -p executor_type:=multi
```

**预期现象**：
```
[INFO] Executor Type: multi
[INFO] Using MultiThreadedExecutor (Parallel!)
[INFO] [心跳] 咚! 线程 ID: 140123456001
[WARN] [计算] 开始复杂计算... 线程 ID: 140123456002 (预计阻塞 2秒)
[INFO] [心跳] 咚! 线程 ID: 140123456001  # ✅ 心跳继续跳动！
[INFO] [心跳] 咚! 线程 ID: 140123456001  # ✅ 心跳继续跳动！
[WARN] [计算] 计算完成！
```

**分析**：
- ✅ 心跳和计算运行在**不同线程**
- ✅ 心跳不受计算阻塞的影响
- ✅ 适合复杂、实时性要求高的系统

---

## 📊 代码核心讲解

### 回调组的创建与分配

```cpp
// blocking_node.cpp

// 创建两个独立的互斥组
callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

// 心跳定时器 → 分配到组 1
heartbeat_timer_ = this->create_wall_timer(
    500ms,
    std::bind(&BlockingNode::on_heartbeat, this),
    callback_group_1_);  // ← 指定回调组

// 繁重计算定时器 → 分配到组 2
heavy_timer_ = this->create_wall_timer(
    3000ms,
    std::bind(&BlockingNode::on_heavy_calculation, this),
    callback_group_2_);  // ← 指定回调组
```

> [!TIP]
> **关键点**：由于两个定时器在**不同的互斥组**，`MultiThreadedExecutor` 可以并行执行它们。

---

### 主程序中的执行器切换

```cpp
// executor_demo_main.cpp

// 通过 ROS 参数控制执行器类型
node->declare_parameter("executor_type", "single");
std::string executor_type = node->get_parameter("executor_type").as_string();

if (executor_type == "multi")
{
    // 多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
}
else
{
    // 单线程执行器（默认）
    rclcpp::spin(node);  // 等同于 SingleThreadedExecutor
}
```

---

## 🎓 实战应用场景

### 机器人系统中的典型应用

| 任务类型 | 实时性要求 | 推荐回调组 | 原因 |
|---|---|---|---|
| 电机控制指令发送 | 高 | 独立的 MutuallyExclusive | 避免被阻塞 |
| 传感器数据发布 | 高 | 独立的 MutuallyExclusive | 保证发布频率 |
| 全局路径规划 | 低 | 独立的 MutuallyExclusive 或 Reentrant | 可以阻塞较长时间 |
| 图像处理 | 低 | 独立的 MutuallyExclusive 或 Reentrant | 耗时较长 |
| 数据库写入 | 低 | 独立的 MutuallyExclusive | 避免阻塞主控流程 |

### 推荐架构

```cpp
// 高优先级任务（实时性关键）
auto realtime_group = create_callback_group(CallbackGroupType::MutuallyExclusive);
motor_cmd_timer_ = create_wall_timer(10ms, ..., realtime_group);

// 低优先级任务（可以阻塞）
auto background_group = create_callback_group(CallbackGroupType::Reentrant);
planning_timer_ = create_wall_timer(1s, ..., background_group);
vision_timer_ = create_wall_timer(500ms, ..., background_group);
```

搭配 `MultiThreadedExecutor` 使用，确保实时任务不受后台任务影响。

---

## 🔍 常见问题

### Q1：为什么用了 `MultiThreadedExecutor` 还是被阻塞？

**A**：检查回调组配置！如果所有回调都在**同一个 MutuallyExclusive 组**（或默认组），即使是多线程执行器也只能串行执行。

**解决方法**：
```cpp
// ❌ 错误：都在默认组
timer1_ = create_wall_timer(100ms, callback1);  // 默认组
timer2_ = create_wall_timer(1s, callback2);     // 默认组

// ✅ 正确：分配到不同组
auto group1 = create_callback_group(CallbackGroupType::MutuallyExclusive);
auto group2 = create_callback_group(CallbackGroupType::MutuallyExclusive);
timer1_ = create_wall_timer(100ms, callback1, group1);
timer2_ = create_wall_timer(1s, callback2, group2);
```

---

### Q2：什么时候用 Reentrant 组？

**A**：当回调之间**完全独立**、没有共享状态时，可以用 Reentrant 组提高并发度。

**示例**：
```cpp
auto parallel_group = create_callback_group(CallbackGroupType::Reentrant);

// 这三个任务完全独立，可以并行运行
vision_sub_ = create_subscription(..., parallel_group);
lidar_sub_ = create_subscription(..., parallel_group);
imu_sub_ = create_subscription(..., parallel_group);
```

---

### Q3：单线程执行器有什么用？

**A**：
- ✅ 调试更简单（不用担心竞态条件）
- ✅ 内存占用更小
- ✅ 适合简单、无实时性要求的节点

---

## 📚 相关学习资源

- [ROS 2 官方文档 - Executors](https://docs.ros.org/en/rolling/Concepts/About-Executors.html)
- [ROS 2 Callback Groups 教程](https://docs.ros.org/en/rolling/How-To-Guides/Using-callback-groups.html)
- 本项目的学习路线图：[`../ros2_learning_behavior_tree/docs/bt_learning_roadmap.md`](../ros2_learning_behavior_tree/docs/bt_learning_roadmap.md)

---

## 💡 总结

| 概念 | 核心要点 |
|---|---|
| **Executor** | 控制节点的运行方式（单线程 vs 多线程） |
| **CallbackGroup** | 控制回调之间的并发规则（互斥 vs 可重入） |
| **关键组合** | `MultiThreadedExecutor` + 多个独立的回调组 = 真正的并发 |

掌握这些概念后，你可以：
- 为机器人系统设计高效的并发架构
- 避免实时任务被阻塞
- 充分利用多核 CPU 的性能
