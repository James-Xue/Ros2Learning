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
./scripts/setup_ubuntu.sh

# 3) 构建
./scripts/build.sh

# 4) 进入新终端或 source
source ./install/setup.bash
```

> 说明：如果你使用的是其他 ROS 2 发行版（如 Jazzy/Humble），请在脚本中修改对应的版本名。
