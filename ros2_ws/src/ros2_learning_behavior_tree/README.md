# ROS 2 BehaviorTree.CPP Learning Package

本项目是一个用于学习和演示 **BehaviorTree.CPP (v4)** 与 **ROS 2** 集成的示例包。

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

## � 节点详细说明

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

## 详细教程
更多概念解释请参考：[docs/bt_learning_roadmap.md](docs/bt_learning_roadmap.md)
