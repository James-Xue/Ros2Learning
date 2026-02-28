# ros2_learning_arm_basics

> 基于 MoveIt 2 的 Panda 机械臂控制入门包，涵盖关节空间/笛卡尔空间运动规划、夹爪控制以及带碰撞检测的完整 pick-and-place 流程。

## 功能描述

本包通过 `ArmPositionController` 节点展示 MoveIt 2 `MoveGroupInterface` 的核心用法：

- **命名姿态规划**：调用 SRDF 中预定义的 `ready`/`open`/`close` 等姿态
- **笛卡尔空间规划**：`moveToPose()` 使用 IK，`drawSquare()` 使用 `computeCartesianPath()` 强制直线运动
- **关节空间规划**：`moveJoints()` 直接指定 7 个关节目标角度
- **夹爪控制**：通过 `hand` 规划组控制 Panda 双指夹爪的开合与宽度
- **规划场景管理**：通过 `PlanningSceneInterface` 生成/移除碰撞物体（桌面 + 目标方块）
- **物体附加/分离**：`attachObjectToGripper` / `detachObjectFromGripper` 实现随夹爪移动的真实抓取仿真
- **ACM 修改**：通过发布 `planning_scene` 话题动态修改允许碰撞矩阵，避免抓取时的误检

**当前 `main` 函数实际执行的演示**：仅运行 `runRealisticPickAndPlace()`（完整真实物体抓取放置流程）；其余演示（`runDemo`、`drawSquare`、`runPickAndPlaceDemo`）已在代码中注释掉，需手动解注释后重新编译。

## 📋 简介

这个包提供了机械臂控制的入门示例，帮助ROS2初学者理解MoveIt 2框架的基本用法。

## 🎯 功能特性

- ✅ 移动到预定义姿态（named targets）
- ✅ 笛卡尔空间位置控制
- ✅ 关节角度直接控制
- ✅ 笛卡尔路径规划（画正方形演示）
- ✅ **夹爪控制**（打开/闭合/自定义宽度）
- ✅ **物体生成和管理**（在规划场景中生成碰撞物体）
- ✅ **物理附加/分离**（物体附加到夹爪，实现真实抓取）
- ✅ **真实物体抓取演示**（完整的物体生命周期管理）
- ✅ 自动演示序列
- ✅ 详细的中文注释

## 📦 依赖安装

```bash
# 更新软件源
sudo apt update

# 安装MoveIt 2核心框架
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-plugins \
  ros-jazzy-rviz-visual-tools

# 安装Panda机械臂配置
sudo apt install -y \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-moveit-resources-panda-description

# 安装控制器（必需）
sudo apt install -y \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-controller-manager \
  ros-jazzy-gripper-controllers \
  ros-jazzy-position-controllers
```

## 🚀 快速开始

### 1. 编译包
```bash
cd ~/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_arm_basics
source install/setup.bash
```

### 2. 一键启动演示（推荐）
```bash
# 启动仿真环境和控制节点
ros2 launch ros2_learning_arm_basics demo.launch.py
```

这个命令会自动：
- ✅ 启动 MoveIt 2 仿真环境（`moveit_resources_panda_moveit_config/demo.launch.py`）
- ✅ 打开 RViz2 可视化界面
- ✅ 延迟 5 秒后启动机械臂控制节点（等待 MoveGroup 就绪）
- ✅ **执行真实物体抓取演示**（生成桌面和方块 → 预抓取位置 → 下降 → 夹取 → 提升 → 放置 → 返回 ready）

> 注意：当前 `main` 函数只执行 `runRealisticPickAndPlace()`。如需运行基础动作、正方形或夹爪演示，需在 `arm_position_controller_main.cpp` 中手动解注释对应调用后重新编译。

### 3. 手动启动（可选）

如果需要分别启动各个组件：

```bash
# 终端1：启动MoveIt demo
cd ~/Ros2Learning/ros2_ws
./start_panda_simulation.sh

# 终端2：运行控制节点
cd ~/Ros2Learning/ros2_ws
source install/setup.bash
ros2 run ros2_learning_arm_basics arm_position_controller
```

机械臂会自动执行演示动作序列！

## 📚 文档

- **[快速启动指南](../../ARM_QUICKSTART.md)** - 详细的安装和使用说明
- **[MoveIt深度解析](docs/MOVEIT_DEEP_DIVE.md)** - 深入理解MoveIt框架原理
- **[学习计划](../../ARM_SIMULATION_LEARNING_PLAN.md)** - 8-12周完整学习路线

## 📁 目录结构

```
ros2_learning_arm_basics/
├── CMakeLists.txt
├── package.xml
├── README.md                           # 本文件
├── launch/
│   └── demo.launch.py                 # 一键启动launch文件
├── src/
│   ├── arm_position_controller.cpp    # 位置控制器实现
│   └── arm_position_controller_main.cpp # 主函数
├── include/
│   └── ros2_learning_arm_basics/
│       └── arm_position_controller.hpp # 头文件
└── docs/
    ├── README.md                       # 文档索引
    └── MOVEIT_DEEP_DIVE.md            # MoveIt深度解析
```

## 🎓 核心代码示例

### 移动到预定义姿态
```cpp
moveToNamedTarget("ready");  // 移动到ready姿态
```

### 笛卡尔空间控制
```cpp
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.5;
target_pose.position.y = 0.2;
target_pose.position.z = 0.7;
target_pose.orientation.w = 1.0;
moveToPose(target_pose);
```

### 关节空间控制
```cpp
std::vector<double> joint_values = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
moveJoints(joint_values);
```

### 夹爪控制
```cpp
// 打开夹爪（3.5cm）
openGripper();

// 闭合夹爪
closeGripper();

// 设置自定义宽度（2cm）
setGripperWidth(0.02);
```

### 完整抓取序列
```cpp
// 运行完整的抓取和放置演示
runPickAndPlaceDemo();
```

### 物体管理
```cpp
// 在场景中生成目标物体（5cm立方体）
spawnTargetObject();

// 从场景中移除物体
removeTargetObject();
```

### 真实物体抓取
```cpp
// 将物体附加到夹爪（抓取时）
attachObjectToGripper("target_box");

// 从夹爪分离物体（放置时）
detachObjectFromGripper("target_box");

// 运行完整的真实物体抓取演示
runRealisticPickAndPlace();
```

## 🔧 常见问题

### Q: RViz2窗口打不开？
A: 确保系统支持图形化界面。如果是远程服务器，需要配置X11转发。

### Q: 机械臂规划失败？
A: 检查目标位置是否在工作空间内，尝试调整目标位置或增加规划时间。

### Q: 缺少joint_trajectory_controller？
A: 运行安装命令中的控制器安装步骤。

## 输入/输出

### 话题

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/planning_scene` | `moveit_msgs/msg/PlanningScene` | 动态修改 ACM（Allowed Collision Matrix） |

> 运动规划通过 MoveIt 2 的 Action/Service 接口（`/move_action`、`/compute_ik` 等）完成，由 `MoveGroupInterface` 内部管理，不直接体现为节点话题。

### MoveIt 2 规划组

| 规划组 | 用途 |
|---|---|
| `panda_arm` | 7 自由度机械臂，末端执行器链接为 `panda_hand` |
| `hand` | Panda 双指夹爪，预定义姿态 `open`/`close` |

## 验收测试

暂无，待补充。

## 已知限制

- 依赖 `moveit_resources_panda_moveit_config`，必须先安装 MoveIt 2 及其 Panda 资源包（见"依赖安装"节）。
- 在无 GPU 或图形栈的纯终端/CI 环境下，RViz 会崩溃导致整个 launch 退出。
- `main` 函数中其余演示（`runDemo`、`drawSquare`、`runPickAndPlaceDemo`）已注释掉；README 中的功能特性列表描述的是代码能力，而非当前默认运行的演示内容。
- `runRealisticPickAndPlace()` 中使用 `rclcpp::sleep_for` 进行阻塞式等待，节点在等待期间不处理其他回调。
- 规划成功率依赖 MoveIt 2 随机采样，在特定工作空间边界位姿下可能需要多次重试。

## 相关链接

- **MoveIt 2官方文档**: https://moveit.picknik.ai/jazzy/
- **Panda机械臂**: https://www.franka.de/
- **本项目GitHub**: https://github.com/James-Xue/Ros2Learning

## 许可证

Apache-2.0
