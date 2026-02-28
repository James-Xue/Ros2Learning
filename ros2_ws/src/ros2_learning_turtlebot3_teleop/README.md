# ros2_learning_turtlebot3_teleop

> 用 C++ 实现的键盘遥控节点，以固定频率向 `/cmd_vel` 发布 `TwistStamped` 并在超时时自动刹停。

## 功能描述

本包实现了一个键盘遥控节点（`teleop_keyboard`），允许用户通过 WASD 等按键实时控制差速驱动机器人（如 TurtleBot3）的线速度和角速度。节点使用 `geometry_msgs/msg/TwistStamped`（而非旧版 `Twist`）作为输出，与 ROS 2 Jazzy 时代的控制器接口保持一致。

本节点的核心设计与一般键盘遥控不同，采用**"定时器固定频率发布 + 超时自动刹停"**策略，而非仅在按键时才发送。这样下游控制器能稳定接收周期性指令，也避免因终端暂停或网络中断导致机器人失控持续运动。键盘读取通过 POSIX `select()` 非阻塞探测，不会阻塞 ROS executor 的事件循环。

终端 raw mode（`termios` 的 `ICANON`/`ECHO` 关闭）由 `TerminalRawMode` 类通过 PImpl 封装管理，做到平台相关的系统调用不污染业务头文件。RAII 析构保证退出时终端状态被恢复，不会留下"输入不回显"的副作用。

节点还提供运行时动态调速功能：通过 `q/e` 同步缩放线速度和角速度，通过 `r/f` 单独调线速度，通过 `t/g` 单独调角速度，所有缩放均在超时计时内有效更新，防止触发刹停。

## 运行命令

```bash
# 1. source 环境
source /opt/ros/jazzy/setup.bash
source /root/Ros2Learning/ros2_ws/install/setup.bash

# 2. 直接运行（须在交互终端中执行，不可通过 launch 重定向到日志文件）
ros2 run ros2_learning_turtlebot3_teleop teleop_keyboard

# 3. 指定自定义话题（例如配合命名空间或差速控制器）
ros2 run ros2_learning_turtlebot3_teleop teleop_keyboard \
  --ros-args -p cmd_vel_topic:=/turtlebot3/cmd_vel

# 4. 调整初始速度缩放和发布频率
ros2 run ros2_learning_turtlebot3_teleop teleop_keyboard \
  --ros-args \
    -p linear_scale:=0.15 \
    -p angular_scale:=2.0 \
    -p publish_rate_hz:=10.0 \
    -p command_timeout_s:=1.0

# 5. 验证速度指令正在发布
ros2 topic echo /cmd_vel
```

启动后终端会打印按键说明。按 `?` 可随时重新显示帮助。

## 输入/输出

### 发布的话题（Publisher）

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/cmd_vel`（可通过参数 `cmd_vel_topic` 修改） | `geometry_msgs/msg/TwistStamped` | 以固定频率发布的速度指令；超时时发布全零刹停消息 |

消息中 `twist.linear.x` 为前进速度（m/s），`twist.angular.z` 为角速度（rad/s）。`header.stamp` 在每次发布前实时填充。

### 按键映射

| 按键 | 效果 |
|---|---|
| `w` / `W` | 前进（`linear.x = linear_scale_`） |
| `s` / `S` | 后退（`linear.x = -linear_scale_`） |
| `a` / `A` | 左转（`angular.z = angular_scale_`） |
| `d` / `D` | 右转（`angular.z = -angular_scale_`） |
| `x` / `X` / 空格 | 立即停止 |
| `q` / `Q` | 线速度和角速度同步放大 10% |
| `e` / `E` | 线速度和角速度同步缩小 10% |
| `r` / `R` | 仅线速度放大 10% |
| `f` / `F` | 仅线速度缩小 10% |
| `t` / `T` | 仅角速度放大 10% |
| `g` / `G` | 仅角速度缩小 10% |
| `?` | 打印按键帮助信息 |
| `Ctrl-C` | 退出节点，恢复终端 |

### 可配置的节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `cmd_vel_topic` | string | `/cmd_vel` | 速度指令发布目标话题 |
| `linear_scale` | double | `0.22` | 初始线速度缩放（m/s），TurtleBot3 Waffle Pi 建议值 |
| `angular_scale` | double | `2.84` | 初始角速度缩放（rad/s） |
| `publish_rate_hz` | double | `20.0` | 定时器发布频率（Hz） |
| `command_timeout_s` | double | `0.5` | 无新输入超时时间（秒），超时后发布全零刹停 |

## 验收测试

暂无，待补充。

后续可添加的测试方向：
- 单元测试：验证 `process_keyboard()` 对各按键的速度映射正确性（需要 mock STDIN）
- 单元测试：验证超时逻辑在 `command_timeout_s` 后输出全零消息
- 集成测试：验证节点能在非 TTY 环境（`isatty` 返回 false）下安全降级运行，不崩溃

```bash
# 当测试存在后，运行方式：
cd /root/Ros2Learning/ros2_ws
colcon test --packages-select ros2_learning_turtlebot3_teleop
colcon test-result --verbose
```

## 已知限制

- **必须在交互终端（TTY）中运行**：终端 raw mode 依赖 `isatty(STDIN_FILENO)` 返回真。通过 `ros2 launch` 的 `output='screen'` 重定向后，`raw mode` 将无法启用，键盘输入不可用，节点会打印警告并继续运行（但实际接收不到任何按键）。
- **消息类型为 `TwistStamped`**：部分旧版差速控制器或 `turtlebot3_bringup` 订阅 `geometry_msgs/msg/Twist`（无时间戳版本），直接使用本节点可能导致话题类型不匹配。需根据实际控制器接口选择。
- **无 launch 文件**：本包目前没有提供 `.launch.py`，只能通过 `ros2 run` 启动；在多节点场景中需手动配合其他包的 launch 文件使用。
- **单字节键盘读取**：当前实现每次只读 1 个字节，不支持方向键（ESC 序列，多字节）等特殊键的识别。
- **速度缩放无上限保护**：`q/r/t` 持续放大速度时没有上限夹紧，若长时间按下可能使速度超出硬件安全范围，需用户自行注意。
