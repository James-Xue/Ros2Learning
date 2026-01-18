# ros2_learning_arm_basics

ROS2机械臂控制基础学习包 - 使用MoveIt 2控制Panda机械臂

## 📋 简介

这个包提供了机械臂控制的入门示例，帮助ROS2初学者理解MoveIt 2框架的基本用法。

## 🎯 功能特性

- ✅ 移动到预定义姿态（named targets）
- ✅ 笛卡尔空间位置控制
- ✅ 关节角度直接控制
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
- ✅ 启动 MoveIt 2 仿真环境
- ✅ 打开 RViz2 可视化界面
- ✅ 启动机械臂控制节点
- ✅ 执行演示动作序列

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

## 🔧 常见问题

### Q: RViz2窗口打不开？
A: 确保系统支持图形化界面。如果是远程服务器，需要配置X11转发。

### Q: 机械臂规划失败？
A: 检查目标位置是否在工作空间内，尝试调整目标位置或增加规划时间。

### Q: 缺少joint_trajectory_controller？
A: 运行安装命令中的控制器安装步骤。

## 🌐 相关链接

- **MoveIt 2官方文档**: https://moveit.picknik.ai/jazzy/
- **Panda机械臂**: https://www.franka.de/
- **本项目GitHub**: https://github.com/James-Xue/Ros2Learning

## 📄 许可证

Apache-2.0

## 👨‍💻 作者

James Xue - 学习ROS2机械臂控制
