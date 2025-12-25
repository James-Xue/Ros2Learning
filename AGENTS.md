# Repository Guidelines

## 项目背景与目标
- 这是一个机器人小车项目，使用 ROS + CMake + C++ 编写，运行于 Linux 端。
- 项目用于学习 ROS 相关技术，不以盈利为目的，目标是促进仓库开发者的技术成长。
- 项目仅在单台 Linux 机器上运行，强调便于学习与本地调试。
- 项目由多个 ROS 2 包组成，使用 `colcon` 统一管理与运行。

## 项目结构与模块组织
- `ros2_ws/` 是本项目的 ROS 2 `colcon` 工作空间。
- `ros2_ws/src/` 放置 ROS 2 包（在此克隆或创建包）；每个包对应一个子目录，承担不同职责。
- `ros2_ws/scripts/` 为安装依赖、构建和 source 环境的辅助脚本。
- `ros2_ws/build/`、`ros2_ws/install/`、`ros2_ws/log/` 为 `colcon` 生成的构建产物。

## 构建、测试与开发命令
- `./ros2_ws/scripts/setup_ubuntu.sh jazzy` 安装 Ubuntu 常用依赖并执行 `rosdep`。
- `./ros2_ws/scripts/build.sh jazzy` source `/opt/ros/<distro>` 后运行 `colcon build --symlink-install`。
- `source ./ros2_ws/scripts/source.sh jazzy` 在当前 shell 中叠加工作空间环境。
- `colcon build --symlink-install` 为进入 `ros2_ws/` 后的手动构建命令。
- `colcon test` 运行包测试（如存在）；建议随后执行 `colcon test-result --verbose`。
- 示例运行：`ros2 run ros2_learning_cpp talker`（构建与 source 后执行）。

## 编码风格与命名约定
- 脚本保持 Bash 风格，与现有 `ros2_ws/scripts/*.sh` 一致（含 `set -euo pipefail`）。
- Shell/YAML 建议 2 空格缩进，C++/Python 建议 4 空格缩进，除非包内另有约定。
- ROS 2 包名使用小写加下划线（如 `ros2_learning_cpp`）。
- 节点与文件命名保持可读性（如 `talker`、`listener`、`*_node`）。

## 测试指南
- 当前仓库尚未提供测试；新增包时请按 ROS 2/ament 方式添加测试。
- 使用 `colcon test` 运行测试，并统一命名为 `test_*` 或 `*_test`。

## 提交与 PR 指南
- 历史提交消息较短且带编号（如 `1. ...`）；提交时保持简洁、可检索。
- PR 建议包含：目的、已执行的构建/测试命令、使用的 ROS 发行版（如 `jazzy`）。
- 评审与沟通应直截了当，优先指出问题与改进路径。

## 安全与配置提示
- 脚本默认从 `/opt/ros/<distro>` 读取 ROS 2；可通过参数或 `ROS_DISTRO` 指定。
- `setup_ubuntu.sh` 需要 `sudo` 并会修改系统环境，仅在 Ubuntu 主机上执行。
- 提示：与用户沟通时尽量使用中文回复。
- 提示：进行代码评审时保持专业，但要极端严格、直指问题本身并提供尖锐的技术批评。
