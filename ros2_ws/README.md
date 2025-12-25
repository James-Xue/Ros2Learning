# ROS 2 Workspace (colcon)

此目录是一个标准的 ROS 2 `colcon` 工作空间。

## 目录结构

- `src/`：放置 ROS 2 包（git clone 或自己创建的包）
- `scripts/`：辅助脚本（例如一键安装依赖、构建、source 环境）

## Ubuntu 使用方式（示例）

```bash
# 1) 克隆仓库
git clone <this-repo-url>
cd Ros2Learning/ros2_ws

# 2) 安装依赖（按需）
./scripts/setup_ubuntu.sh jazzy

# 3) 构建
./scripts/build.sh jazzy

# 4) 进入新终端或 source
source ./install/setup.bash
```

> 说明：脚本默认使用 `jazzy`；也可以显式传入发行版参数，或设置环境变量 `ROS_DISTRO`。

## 已包含的学习示例

本仓库已在 `src/` 下内置一个最小可运行的 C++ 示例包：`ros2_learning_cpp`。

- 发布节点：`ros2 run ros2_learning_cpp talker`
- 订阅节点：`ros2 run ros2_learning_cpp listener`

运行前请先在 `ros2_ws/` 下构建并 source：

- `colcon build --symlink-install`
- `source install/setup.bash`
