# ros2_learning_behavior_tree

> 基于 BehaviorTree.CPP v4 的任务调度学习包，演示行为树节点注册、黑板数据流、子树组合与 Nav2 动作客户端集成，支持 mock 模式（无需 Nav2）和真实导航模式。

## 功能描述

本包通过构建多个可运行的行为树 XML 场景，演示以下核心概念：

- **工厂模式注册**：在 `bt_main.cpp` 中以 `registerBuilder` + lambda 捕获 ROS 节点指针的方式注册所有自定义节点，解决异构依赖注入问题。
- **BT 节点类型对比**：`SyncActionNode`（单次 tick 必须完成）vs `StatefulActionNode`（可返回 RUNNING 的长耗时节点）。
- **黑板（Blackboard）数据流**：`GetLocationFromQueue` 作为生产者将地点名写入黑板，`MockMoveBase` 作为消费者读取并执行。
- **容错（Fallback）逻辑**：`mock_fallback_demo.xml` 演示低成功率任务失败后触发恢复动作并重试的完整流程。
- **子树（SubTree）组合**：`main_tree_composition.xml` 调用 `fetch_subtree.xml` 中定义的可复用逻辑单元。
- **Nav2 真实导航**：`MoveBase` 节点封装 `navigate_to_pose` 动作客户端，通过 `simple_patrol.xml` 在仿真/实体机器人上巡逻。
- **Groot2 实时监控**：进程启动后自动在端口 1667 发布树状态，可用 Groot2 可视化调试。

## 运行依赖 (重要)

本项目设计了两种运行模式，请根据您的环境选择：

1.  **Mock (模拟) 模式 [推荐入门]**:
    *   **无需独立安装 Nav2**。
    *   使用打桩过的 C++ 节点模拟移动和恢复逻辑。
    *   **适用场景**：纯逻辑验证、学习黑板数据流、学习容错机制。
2.  **Real (真实) 模式**:
    *   **需要安装并启动 Nav2**（提供 `/navigate_to_pose` 动作服务器）。
    *   **适用场景**：真实机器人或 Gazebo 仿真环境。

---

## 编译与安装

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_learning_behavior_tree
source install/setup.bash
```

---

## 启动演示 (Mock 逻辑验证)

如果您没有运行 Nav2，请务必启动带 `mock` 字样的行为树，否则程序会因找不到 Action Server 而退出。

### 1. 容错逻辑演示 (Fallback)
演示一个低成功率的任务，失败后执行恢复动作并重试。
```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=mock_fallback_demo.xml
```

### 2. 数据流演示 (Blackboard)
演示如何从队列取点，通过黑板传递给移动节点。
```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=mock_blackboard_demo.xml
```

### 3. 模块化子树演示 (SubTree)
演示主树如何加载并调用外部定义的子树文件。
```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=main_tree_composition.xml
```

---

## 启动演示 (真实 Nav2 环境)

**注意**：运行以下命令前，请确保您的机器人仿真环境已启动，且 Nav2 节点正常工作。

```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=simple_patrol.xml
```

---

## 节点详细说明

本项目采用了 **“接口与实现分离”** 的工程化结构（`.hpp` 定义接口，`.cpp` 编写逻辑），并在 `bt_main.cpp` 中通过工厂模式统一注册。

### 1. 模拟 (Mock) 节点 - 逻辑测试专用

| 节点名 | 基类 | 端口 (Ports) | 详细逻辑说明 |
| :--- | :--- | :--- | :--- |
| **`GetLocationFromQueue`** | `SyncActionNode` | `target_location` (Out) | **生产者**。依次从内部队列（Kitchen, Bedroom...）中取出一个地点名称并写入黑板。当队列遍历完后会自动循环。 |
| **`MockMoveBase`** | `StatefulActionNode` | `location` (In)<br>`probability` (In)<br>`duration` (In) | **耗时模拟**。根据 `duration` 参数维持 `RUNNING` 状态。倒计时结束后，通过随机数结合 `probability` (0.0~1.0) 决定返回 `SUCCESS` 还是 `FAILURE`。 |
| **`MockRecovery`** | `StatefulActionNode` | `type` (In)<br>`duration` (In) | **故障恢复**。模拟机器人报错后的恢复动作（如旋转、清理地图）。始终在耗费 `duration` 时长后返回 `SUCCESS`。 |

### 2. 真实 (Real) 节点 - 物理/仿真对接

| 节点名 | 基类 | 端口 (Ports) | 详细逻辑说明 |
| :--- | :--- | :--- | :--- |
| **`MoveBase`** | `StatefulActionNode` | `goal_x` (In)<br>`goal_y` (In)<br>`goal_yaw` (In) | **真实导航**。封装了 ROS 2 `navigate_to_pose` 动作客户端。它会异步发送目标位姿，并在导航期间持续汇报 `RUNNING`。 |
| **`SimpleArmAction`** | `SyncActionNode` | `target_joint_angle` (In) | **动作打桩**。模拟机械臂关节控制。目前仅通过 `rclcpp::Rate` 进行短时间阻塞模拟执行成功。 |

---

## 可视化调试 (Groot2)

本包已集成 Groot2 发布器。启动任意行为树后，可以使用 Groot2 进行实时监控：
1. 下载并打开 [Groot2](https://www.behaviortree.dev/groot)。
2. 点击 **Connect** -> **Standard Mode**。
3. 输入 `localhost`，点击连接即可看到当前树的实时状态（变绿色代表运行中）。

---

## 输入/输出

本包为纯执行端，不发布话题也不提供服务。与外部系统的接口均通过 BT 节点内部封装：

| 方向 | 接口 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| 动作客户端（输出）| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `MoveBase` 节点（真实模式）向 Nav2 发送导航目标 |

Mock 节点（`MockMoveBase`、`MockRecovery`、`GetLocationFromQueue`）不与任何外部话题/服务交互，所有数据交换通过 BT 黑板完成。

### 黑板端口汇总

| 节点 | 端口名 | 方向 | 类型 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `GetLocationFromQueue` | `target_location` | Out | `std::string` | 依次输出队列中的地点名称 |
| `MockMoveBase` | `location` | In | `std::string` | 读取目标地点名称（仅打日志用） |
| `MockMoveBase` | `probability` | In | `double` | 模拟成功概率（0.0~1.0） |
| `MockMoveBase` | `duration` | In | `double` | 模拟执行耗时（秒） |
| `MockRecovery` | `type` | In | `std::string` | 恢复动作类型标识（如 "rotate"） |
| `MockRecovery` | `duration` | In | `double` | 模拟恢复耗时（秒） |
| `MoveBase` | `goal_x` | In | `double` | 导航目标 X 坐标（米） |
| `MoveBase` | `goal_y` | In | `double` | 导航目标 Y 坐标（米） |
| `MoveBase` | `goal_yaw` | In | `double` | 导航目标偏航角（弧度） |

## 验收测试

```bash
# 在 ros2_ws/ 目录下执行
colcon build --packages-select ros2_learning_behavior_tree
colcon test --packages-select ros2_learning_behavior_tree
colcon test-result --verbose
```

测试目标：`test_simple_patrol`（gtest）

测试内容：以 `simple_patrol.xml` 为输入，验证工厂能够正确注册 `MoveBase` 和 `SimpleArmAction` 节点并成功实例化行为树（无异常抛出即通过）。

**注意**：测试不运行 ROS 2 spin 循环，也不连接真实 Nav2，仅验证节点注册与树实例化的正确性。

## 已知限制

- `simple_patrol.xml`（真实模式）在没有 Nav2 动作服务器的环境下，`MoveBase` 节点会在 `onStart()` 阶段无限等待服务器上线，程序不会报错退出，而是卡死——应改为设置等待超时。
- Groot2 发布器硬编码端口 1667，多实例并发启动时会端口冲突。
- `bt_main.cpp` 的 tick 循环以 10 Hz 固定频率运行，未根据实际 BT 节点的时间特征自适应调整，会引入最多 100 ms 的额外响应延迟。
- Mock 节点通过 `std::this_thread::sleep_for` 模拟耗时，会在单线程执行器下阻塞 `rclcpp::spin_some`，导致同一进程内其他 ROS 回调被延迟。

## 详细教程
更多概念解释请参考：[docs/bt_learning_roadmap.md](docs/bt_learning_roadmap.md)
