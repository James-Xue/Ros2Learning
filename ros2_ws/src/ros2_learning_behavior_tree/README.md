# ROS2 Learning Behavior Tree

这是一个 ROS2 行为树（Behavior Tree）学习示例包，展示了如何使用 **BehaviorTree.CPP** 库与 ROS2 集成，实现复杂的机器人任务规划。

## 📦 包概述

本包实现了一个简单的"巡逻-抓取-投放"任务，结合了：
- **移动底盘导航**（通过 Nav2 的 `NavigateToPose` Action）
- **机械臂控制**（模拟的关节运动）

## 🏗️ 核心架构

### BehaviorTree.CPP 集成模式

```
BehaviorTree.CPP (核心库)
    ↓
自定义节点封装 (MoveBase, SimpleArmAction)
    ↓
ROS2 接口 (Action Client, Logger)
```

### 节点类型

| 节点类 | 基类 | 用途 | 执行模式 |
|--------|------|------|----------|
| `MoveBase` | `StatefulActionNode` | 导航到目标点 | 异步（支持 RUNNING） |
| `SimpleArmAction` | `SyncActionNode` | 机械臂运动 | 同步（立即完成） |

---

## 📂 项目结构

```
ros2_learning_behavior_tree/
├── behavior_trees/           # 行为树 XML 定义
│   └── simple_patrol.xml     # 巡逻任务树
├── include/ros2_learning_behavior_tree/nodes/
│   ├── move_base_node.hpp    # 导航节点头文件
│   └── simple_arm_action.hpp # 机械臂节点头文件
├── src/
│   ├── bt_executor.cpp       # 行为树执行器主程序
│   └── nodes/                # 节点实现
│       ├── move_base_node.cpp
│       └── simple_arm_action.cpp
├── launch/
│   └── bt_demo.launch.py     # 启动文件
├── test/                     # 单元测试
├── CMakeLists.txt
└── package.xml
```

---

## 🎯 核心概念

### 1. **行为树节点类型**

#### **SyncActionNode（同步节点）**
- **特点**：`tick()` 必须立即返回 `SUCCESS` 或 `FAILURE`
- **适用**：瞬时操作（设置参数、简单计算）
- **示例**：`SimpleArmAction`

```cpp
class SimpleArmAction : public BT::SyncActionNode {
    BT::NodeStatus tick() override {
        // 快速执行，立即返回
        return BT::NodeStatus::SUCCESS;
    }
};
```

#### **StatefulActionNode（状态节点）**
- **特点**：可返回 `RUNNING`，支持多次 tick
- **适用**：长时任务（导航、等待）
- **示例**：`MoveBase`

```cpp
class MoveBase : public BT::StatefulActionNode {
    BT::NodeStatus onStart() override {
        // 发起异步任务
        return BT::NodeStatus::RUNNING;
    }
    
    BT::NodeStatus onRunning() override {
        // 检查进度
        if (任务完成) return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::RUNNING;
    }
    
    void onHalted() override {
        // 清理资源
    }
};
```

### 2. **端口系统（Port System）**

端口是行为树节点之间传递数据的接口：

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("goal_x", "目标点的 X 坐标"),
        BT::InputPort<double>("goal_y", "目标点的 Y 坐标"),
        BT::InputPort<double>("goal_yaw", "目标点的偏航角")
    };
}
```

在 XML 中使用：

```xml
<!-- 硬编码值 -->
<MoveBase goal_x="1.0" goal_y="0.0" goal_yaw="0.0"/>

<!-- 从黑板读取 -->
<MoveBase goal_x="{target_x}" goal_y="{target_y}" goal_yaw="{target_yaw}"/>
```

### 3. **控制节点**

- **Sequence（序列节点）**：按顺序执行，任意失败则整体失败
- **Fallback（后备节点）**：按顺序执行，任意成功则整体成功
- **Parallel（并行节点）**：同时执行多个子节点

---

## 🚀 快速开始

### 1. **编译包**

```bash
cd ~/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_behavior_tree
source install/setup.bash
```

### 2. **运行示例**

```bash
# 启动行为树节点
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py
```

### 3. **查看日志**

观察终端输出，您会看到：
```
[bt_executor]: MoveBase: 发送导航目标 (1.00, 0.00, 0.00)
[bt_executor]: MoveBase: 服务器已接收目标，正在执行...
[bt_executor]: SimpleArmAction: 模拟机械臂移动到 1.57 rad
[bt_executor]: MoveBase: 导航成功到达！
```

---

## 📝 示例任务：巡逻-抓取-投放

行为树定义（`simple_patrol.xml`）：

```xml
<Sequence name="patrol_and_pick">
    <!-- 1. 导航到抓取点 -->
    <MoveBase goal_x="1.0" goal_y="0.0" goal_yaw="0.0" />
    
    <!-- 2. 机械臂抓取 -->
    <SimpleArmAction target_joint_angle="1.57" />
    
    <!-- 3. 导航到投放点 -->
    <MoveBase goal_x="0.0" goal_y="0.0" goal_yaw="3.14" />
    
    <!-- 4. 机械臂复位 -->
    <SimpleArmAction target_joint_angle="0.0" />
</Sequence>
```

**执行流程：**
1. 机器人导航到 `(1.0, 0.0)`
2. 机械臂移动到 1.57 弧度
3. 机器人返回原点 `(0.0, 0.0)`
4. 机械臂复位到 0 弧度

---

## 🔧 关键技术点

### 1. **ROS Action 的异步集成**

`MoveBase` 展示了如何将异步的 ROS Action 集成到同步的行为树框架中：

```cpp
// 发送异步请求
future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);

// 非阻塞检查状态
BT::NodeStatus onRunning() {
    if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        goal_handle_ = future_goal_handle_.get();
    }
    
    // 从回调中读取结果
    if (nav_result_status_.has_value()) {
        return nav_result_status_.value();
    }
    
    return BT::NodeStatus::RUNNING;
}
```

### 2. **依赖注入模式**

通过构造函数注入 ROS 节点，实现解耦：

```cpp
MoveBase(const std::string& name, 
         const BT::NodeConfig& config,
         rclcpp::Node::SharedPtr node_ptr)  // ← 外部传入
: BT::StatefulActionNode(name, config), node_(node_ptr) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}
```

### 3. **节点注册到 BT 工厂**

```cpp
BT::BehaviorTreeFactory factory;

// 使用 lambda 传递 ROS 节点
factory.registerNodeType<MoveBase>(
    "MoveBase",
    [node](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<MoveBase>(name, config, node);
    }
);
```

---

## 🎓 学习路径

### 初级
1. ✅ 理解行为树基本概念（Sequence、Fallback、Action）
2. ✅ 运行本示例，观察执行流程
3. ✅ 修改 XML 文件，调整任务顺序

### 中级
4. ⚡ 实现自定义 `SyncActionNode`（如发布消息）
5. ⚡ 实现自定义 `ConditionNode`（如检查电池电量）
6. ⚡ 使用黑板系统传递数据

### 高级
7. 🚀 实现带超时的导航节点
8. 🚀 添加错误恢复逻辑（Fallback）
9. 🚀 实现复杂的多机器人协作任务

---

## 📚 依赖项

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>behaviortree_cpp</depend>
<depend>nav2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
```

---

## 🐛 常见问题

### Q1: 行为树一直返回 FAILURE？
**A**: 检查 Nav2 是否启动，`MoveBase` 需要 `navigate_to_pose` Action Server。

### Q2: 如何调试行为树？
**A**: 使用 `RCLCPP_INFO` 在关键位置打印日志，或使用 Groot 可视化工具。

### Q3: SyncActionNode vs StatefulActionNode 如何选择？
**A**: 
- 任务耗时 < 100ms → `SyncActionNode`
- 任务耗时 > 100ms 或需要异步操作 → `StatefulActionNode`

---

## 📖 参考资源

- [BehaviorTree.CPP 官方文档](https://www.behaviortree.dev/)
- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [ROS2 Actions 教程](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

---

## 📄 License

Apache-2.0

---

**作者**: ROS2 学习示例  
**维护者**: user@todo.todo  
**版本**: 0.0.1
