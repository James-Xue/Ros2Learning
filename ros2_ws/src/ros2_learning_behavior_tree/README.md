# 🌳 ROS 2 BehaviorTree.CPP Learning Package

这是一个用于学习和演示 **BehaviorTree.CPP (v4)** 与 **ROS 2** 集成的示例包。

本项目展示了从基础的 Action 节点封装，到高级的**控制流 (Control Flow)**、**数据流 (Blackboard)** 以及 **模块化子树 (SubTree)** 的完整实现。

为了方便学习，本项目包含了一组 **Mock (模拟)** 节点，**不需要** 复杂的仿真环境（如 Nav2/Gazebo）即可在终端中运行完整的逻辑验证。

---

## 🚀 功能特性

- **ROS 2 Action 集成**：演示如何将 `rclcpp_action::Client` 封装为 `BT::StatefulActionNode`。
- **自定义节点**：包含同步节点 (`SyncActionNode`) 和异步节点 (`StatefulActionNode`) 的实现范例。
- **Mock 仿真环境**：提供纯逻辑的模拟节点，支持概率失败、耗时模拟，用于测试行为树的鲁棒性。
- **高级特性演示**：
  - **Fallback (容错)**：导航失败自动重试。
  - **Blackboard (黑板)**：节点间的数据传递（动态目标点）。
  - **SubTree (子树)**：行为树的模块化拆分与复用。

---

## 📦 节点说明

### 1. Mock 节点 (逻辑验证专用)

| 节点名 | 类型 | 端口 (Ports) | 说明 |
| :--- | :--- | :--- | :--- |
| **`MockMoveBase`** | Stateful | `location` (In), `probability` (In), `duration` (In) | 模拟移动到底座。支持设置成功率 (0.0-1.0) 和耗时。用于测试 Fallback 逻辑。 |
| **`MockRecovery`** | Stateful | `type` (In), `duration` (In) | 模拟恢复行为（如原地旋转）。 |
| **`GetLocationFromQueue`** | Sync | `target_location` (Out) | 从内部队列中循环取出一个地点，写入黑板。用于演示数据流。 |

### 2. 真实业务节点 (依赖 Nav2)

| 节点名 | 类型 | 端口 (Ports) | 说明 |
| :--- | :--- | :--- | :--- |
| **`MoveBase`** | Stateful | `goal_x`, `goal_y`, `goal_yaw` (In) | 封装了 Nav2 的 `navigate_to_pose` Action Client。 |
| **`SimpleArmAction`** | Stateful | `target_joint_angle` (In) | 模拟机械臂动作（此处仅为打桩，预留接口）。 |

---

## 🛠️ 编译与运行

### 1. 编译

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_learning_behavior_tree
source install/setup.bash
```

### 2. 运行演示

本包提供了一个通用的 Launch 文件，可以通过参数加载不同的行为树文件。

#### 🟢 演示 A：模块化与数据流 (推荐)
演示内容：主树循环调用子树，通过黑板传递动态目标点。
```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=main_tree_composition.xml
```
*预期行为：机器人依次前往 Kitchen -> Bedroom -> Balcony -> Dock，循环执行。*

#### 🟠 演示 B：容错机制 (Fallback)
演示内容：前往一个低成功率的目标点，失败后触发恢复行为，然后重试。
```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=mock_fallback_demo.xml
```
*预期行为：尝试前往 "Dangerous Zone" -> 失败 -> 执行 "Spin" 恢复 -> 重试成功。*

#### 🔵 演示 C：基础巡逻
演示内容：最简单的顺序执行任务。
```bash
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=simple_patrol.xml
```

---

## 📂 文件结构

```text
.
├── behavior_trees/              # XML 行为树文件
│   ├── main_tree_composition.xml  # [主树] 演示 SubTree 和 Blackboard
│   ├── fetch_subtree.xml          # [子树] 被主树调用
│   ├── mock_fallback_demo.xml     # [演示] 演示 Fallback 容错
│   └── simple_patrol.xml          # [演示] 基础巡逻
├── include/.../nodes/           # C++ 头文件 (拆分规范)
│   ├── mock_move_base.hpp
│   ├── mock_recovery.hpp
│   ├── get_location_from_queue.hpp
│   └── ...
├── src/
│   ├── bt_main.cpp              # 主程序 (Factory 注册与 Tick 循环)
│   └── nodes/                   # 节点实现
└── launch/
    └── bt_demo.launch.py        # 启动脚本
```

## 📝 学习笔记

详细的学习路线图请参考：[docs/bt_learning_roadmap.md](docs/bt_learning_roadmap.md)
