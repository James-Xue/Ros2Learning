# Ros2Learning

A ROS 2 simulation project for an autonomous vacuum robot.

## ROS 2（Ubuntu）快速开始

仓库内提供了一个标准 `colcon` 工作空间：`ros2_ws/`。

### 1) 克隆并进入工作空间

- 进入：`Ros2Learning/ros2_ws`
- ROS 2 包放在：`ros2_ws/src/`

### 2) 安装依赖（可选）

在 Ubuntu 上运行：

- `./scripts/setup_ubuntu.sh jazzy`（默认 jazzy，可改为你的发行版）

> 前提：你的机器已经安装了对应 ROS 2（例如 `/opt/ros/jazzy` 存在）。

### 3) 构建

- `./scripts/build.sh jazzy`

### 4) 使用（source 环境）

在当前终端执行：

- `source ./scripts/source.sh jazzy`

之后即可直接 `ros2 ...` 使用，并且包含本工作空间 overlay。
