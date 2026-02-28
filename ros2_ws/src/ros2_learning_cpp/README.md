# ros2_learning_cpp

> 演示 rclcpp 三大核心机制的最小化示例包：话题发布/订阅、组合节点（Composition）、生命周期节点（Lifecycle Node）。

## 功能描述

本包以"talker 发布 / listener 订阅"为骨架，在一个最小示例上叠加了两个进阶特性。

**话题发布/订阅**：`TalkerNode` 每 500 ms 向 `/chatter` 话题发布一条 `std_msgs/String` 消息，消息内容来自可配置参数 `message_text`，并附带递增计数。`ListenerNode` 订阅同一话题并将收到的内容打印到终端。

**Composition（组合节点）**：两个节点均通过 `RCLCPP_COMPONENTS_REGISTER_NODE` 注册为 rclcpp 组件插件，可被加载到同一个 `component_container` 进程中运行。组合模式下启用 `use_intra_process_comms: true`，消息在进程内通过指针传递，完全绕过序列化/反序列化，达到零拷贝效果。

**Lifecycle Node（生命周期节点）**：`TalkerNode` 继承自 `rclcpp_lifecycle::LifecycleNode` 而非普通 `rclcpp::Node`。它实现了五个生命周期回调：`on_configure`（创建发布者和定时器）、`on_activate`（激活 `LifecyclePublisher`，开始真正发布数据）、`on_deactivate`（停止发布）、`on_cleanup`（销毁资源，回到未配置状态）、`on_shutdown`。在 Inactive 状态下调用 `publish()` 是静默的无效操作，不会发出任何消息，这是 `LifecyclePublisher` 的核心保障机制。

学习要点：如何用 `rclcpp_components_register_node` 将节点注册为组件、进程内通信的开启方式、生命周期状态机的五个关键回调及资源管理时机。

## 运行命令

```bash
# 1. 环境准备（每个终端均需执行）
source /opt/ros/jazzy/setup.bash
source /root/Ros2Learning/ros2_ws/install/setup.bash

# 2. 独立进程模式：分两个终端分别运行
ros2 run ros2_learning_cpp talker
ros2 run ros2_learning_cpp listener

# 3. 通过 launch 文件启动 talker（支持参数传递）
ros2 launch ros2_learning_cpp talker.launch.py message_text:="Hello ROS 2"

# 4. Composition 模式：talker + listener 跑在同一个进程内（进程内通信）
ros2 launch ros2_learning_cpp composition.launch.py

# 5. Lifecycle 演示：talker 以生命周期节点启动，listener 在容器内运行
ros2 launch ros2_learning_cpp lifecycle_demo.launch.py

# --- lifecycle_demo 启动后，在另一个终端手动驱动生命周期状态机 ---
ros2 lifecycle set /talker_lifecycle configure
ros2 lifecycle set /talker_lifecycle activate
# 此时 /chatter 开始有消息输出
ros2 lifecycle set /talker_lifecycle deactivate
ros2 lifecycle set /talker_lifecycle cleanup
ros2 lifecycle set /talker_lifecycle shutdown

# 查询当前生命周期状态
ros2 lifecycle get /talker_lifecycle
```

## 输入/输出

### 话题

| 方向 | 话题名 | 消息类型 | 说明 |
|---|---|---|---|
| 发布 | `/chatter` | `std_msgs/msg/String` | TalkerNode 每 500 ms 发布一条，内容为 `<message_text>, count=<N>`；仅在 Lifecycle Active 状态下实际输出 |
| 订阅 | `/chatter` | `std_msgs/msg/String` | ListenerNode 接收并打印到终端 |

### 节点参数

| 节点 | 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|---|
| `talker_lifecycle` | `message_text` | string | `"Hello Lifecycle World"` | 发布消息的文本前缀 |

### 服务（生命周期管理）

| 服务名 | 类型 | 说明 |
|---|---|---|
| `/talker_lifecycle/change_state` | `lifecycle_msgs/srv/ChangeState` | 驱动生命周期状态转移 |
| `/talker_lifecycle/get_state` | `lifecycle_msgs/srv/GetState` | 查询当前生命周期状态 |

## 验收测试

暂无，待补充。

## 已知限制

- `TalkerNode` 在 `on_configure` 阶段同时创建了定时器，定时器本身在 Inactive 状态下仍在计时；只是 `LifecyclePublisher::publish()` 在未激活时静默丢弃消息，并非真正暂停定时器。如需更严格的资源控制，应在 `on_activate` 中创建定时器，在 `on_deactivate` 中调用 `timer_->cancel()`。
- `lifecycle_demo.launch.py` 将 `talker_lifecycle` 节点的生命周期完全交由用户手动管理，启动后不会自动进入 Active 状态，终端不会立即看到消息输出。
- 本包不提供任何导航或传感器接口，仅用于基础机制学习。
