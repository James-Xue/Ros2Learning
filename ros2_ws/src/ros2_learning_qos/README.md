# ros2_learning_qos

> 通过火星探测车仿真场景，演示 ROS 2 四种核心 QoS 策略（Best Effort、Reliable、Transient Local、Deadline）及 Composition 组件化架构的同进程零拷贝通信。

# ROS 2 QoS (Quality of Service) 深度解析：火星探测车通信实战

本项目通过模拟 **火星探测车 (Mars Rover)** 与 **地球控制中心 (Mission Control)** 之间的通信，演示 ROS 2 基于 DDS 的 QoS 机制。

ROS 2 底层使用 DDS (Data Distribution Service)，默认基于 **UDP** 协议。通过配置不同的 QoS 策略，我们可以实现类似 TCP 的可靠传输，或者类似 UDP 的实时传输，甚至更高级的历史数据保留和存活检测。

## 🪐 业务场景 (Business Scenario)

在火星探测任务中，通信带宽极低且不稳定，不同数据的优先级完全不同：

| 数据类型 | 业务需求 | 推荐 QoS 策略 | 对应 DDS 行为 |
| :--- | :--- | :--- | :--- |
| **实时视频流** | 高频数据，允许偶尔丢帧，但必须保证低延迟。过时的帧没有价值。 | **Best Effort** + Volatile | 类似 UDP，发后即忘。不重传，不确认。 |
| **紧急指令确认** | 必须确保指令（如急停）已送达。如果网络丢包，必须重传。 | **Reliable** + Volatile | 类似 TCP，有 ACK 机制和重传队列。 |
| **静态地图数据** | 地图很少更新。**后加入**的控制台（Late Joiner）必须能收到最近的一份地图。 | **Reliable** + **Transient Local** | 发布者会保留最后 N 条数据，新订阅者连接时自动补发（类似 ROS 1 Latched）。 |
| **系统心跳** | 安全监控。如果 N 毫秒内没收到心跳，视为系统失联，触发报警。 | **Deadline** (600ms) | 如果发布/接收间隔超过设定值，触发事件回调。 |

## 🛠️ 代码结构 (组件化架构)

本项目采用了 ROS 2 推荐的 **Composition (组件化)** 架构，实现了节点的解耦和零拷贝通信：

- **`rover_node.hpp / .cpp`**: 模拟火星车，发布上述 4 种数据。注册为可动态加载的插件。
- **`mission_control_node.hpp / .cpp`**: 模拟控制台，订阅数据并监控 QoS 事件（如 Deadline Missed）。注册为插件。
- **`qos_demo.launch.py`**: 使用 `ComposableNodeContainer` 将上述两个节点加载到**同一个进程**中运行。

通过组件化，我们不仅展示了各种 QoS 策略，还展示了 ROS 2 如何在同一进程内实现高性能的零拷贝通信。

## 🚀 编译与运行

1. **编译**:
   ```bash
   cd ~/Ros2Learning/ros2_ws
   colcon build --packages-select ros2_learning_qos --parallel-workers 2
   source install/setup.bash
   ```

2. **运行演示 (推荐：同进程组件模式)**:
   ```bash
   ros2 launch ros2_learning_qos qos_demo.launch.py
   ```
   > 容器 `component_container` 会将火星车和控制台两个插件加载到同一个进程中运行。

3. **运行演示 (备选：独立进程模式)**:
   如果你想跨进程测试真实的 DDS 网络行为，也可以起两个终端分别运行（向后兼容）：
   ```bash
   ros2 run ros2_learning_qos qos_producer
   ros2 run ros2_learning_qos qos_consumer
   ```

## 🧪 实验现象预期

1. **视频流**: 控制台会收到大量视频数据，日志显示 `[Rover] 发送视频流 (Best Effort)`。如果网络不好，中间会丢包但不会阻塞。
2. **地图数据**: 即使 `MissionControl` 启动较慢，或者中途重启，它一启动就能收到 `>>> [Control] 同步到地图数据 <<<`，因为 Publisher 设置了 `Transient Local`。
3. **心跳监控**: 
   - 正常情况下心跳平稳。
   - 代码中模拟了 **每 10 秒一次的丢包**（跳过 3 轮心跳发布）。
   - 此时你会看到控制台报错：`!!! [ALARM] 心跳丢失！截止期未收到数据 !!!`。这展示了 Deadline QoS 的作用，且不会像 `sleep_for` 那样阻塞整个系统。
4. **不兼容测试**: 代码中尝试用 `Reliable` 订阅 `Best Effort` 的视频流，结果是**收不到任何数据**。这是因为 QoS 兼容性规则（Request > Offer）。

## 📚 关键代码片段 (C++)

```cpp
// 1. 视频流：Best Effort (像 UDP)
auto camera_qos = rclcpp::SensorDataQoS(); 

// 2. 地图：Transient Local (像 Latched)
auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

// 3. 心跳：Deadline (看门狗)
auto heartbeat_qos = rclcpp::QoS(rclcpp::KeepLast(10)).deadline(600ms);
```

通过这个 Demo，你可以直观地理解 ROS 2 如何利用 DDS 的强大特性来应对复杂的网络环境。

## 输入/输出

### RoverNode（发布者）

| 话题 | 消息类型 | QoS 策略 | 说明 |
| :--- | :--- | :--- | :--- |
| `camera/image_raw` | `sensor_msgs/msg/Image` | Best Effort（SensorDataQoS）| 模拟高频视频流，500 ms 一帧 |
| `rover/cmd_ack` | `std_msgs/msg/String` | Reliable，KeepLast(10) | 指令执行确认，必须到达 |
| `rover/map` | `std_msgs/msg/String` | Reliable + Transient Local，KeepLast(1) | 静态地图，节点启动时发布一次，后加入者自动同步 |
| `rover/heartbeat` | `std_msgs/msg/String` | KeepLast(10)，Deadline 600 ms | 心跳信号，超时触发控制台报警 |

### MissionControlNode（订阅者）

| 话题 | 消息类型 | QoS 策略 | 说明 |
| :--- | :--- | :--- | :--- |
| `camera/image_raw` | `sensor_msgs/msg/Image` | Best Effort（SensorDataQoS）| 兼容接收视频帧 |
| `rover/cmd_ack` | `std_msgs/msg/String` | Reliable，KeepLast(10) | 打印收到的指令确认 |
| `rover/map` | `std_msgs/msg/String` | Reliable + Transient Local，KeepLast(1) | 迟加入时自动补发地图 |
| `rover/heartbeat` | `std_msgs/msg/String` | KeepLast(10)，Deadline 600 ms | 超时触发 `deadline_callback` 报警 |
| `camera/image_raw`（不兼容测试）| `sensor_msgs/msg/Image` | Reliable（故意与发布者不兼容）| 触发 `incompatible_qos_callback`，验证收不到数据 |

本包不对外提供任何服务或动作接口。

## 验收测试

本包暂无自动化测试目标。验收方式为手动运行并观察控制台输出：

```bash
# 编译
cd /root/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_qos

# 启动演示
source install/setup.bash
ros2 launch ros2_learning_qos qos_demo.launch.py
```

预期观察到以下输出（缺少任何一项均为异常）：

1. `[Rover] 地图数据已发布 (Transient Local)` —— 启动即出现
2. `>>> [Control] 同步到地图数据 <<<` —— MissionControl 加载后立即出现
3. `[Control] 收到指令确认: CMD_EXECUTED_OK` —— 每 500 ms 一次
4. `!!! [ALARM] 心跳丢失！` —— 约每 10 秒触发一次
5. `[Control] QoS 不兼容!` —— 启动后出现，确认不兼容检测生效

暂无 `colcon test` 目标，待补充。

## 已知限制

- `rover_node.cpp` 的 `skip_heartbeat_rounds_` 逻辑在跳过心跳期间也会跳过视频帧和指令确认的发布，与代码注释"仅心跳逻辑挂了"的描述不一致，实为整条回调被跳过。
- 不兼容 QoS 测试中，`incompatible_sub_` 订阅 `camera/image_raw` 时使用 Reliable，与 Best Effort 发布者不兼容是预期行为，但 `incompatible_qos_callback` 依赖 rmw 实现触发，在部分 rmw 后端（如 `rmw_cyclonedds`）可能不触发警告。
- 演示代码硬编码了 Deadline 600 ms 和跳过计数器（每 20 次 tick 触发一次），无法通过 ROS 参数动态调整，不适合作为真实系统模板。
