# ros2_learning_turtlesim_go_to

> 通过自定义服务 `/ros2_learning/turtlesim/go_to` 以比例控制（P-controller）驱动 turtlesim 海龟平滑运动到目标位姿。

## 功能描述

本包包含两个可执行节点：

- **`turtlesim_go_to_server`**：提供 `/ros2_learning/turtlesim/go_to` 服务。接到请求后，订阅 `/<turtle_name>/pose` 获取当前位姿，向 `/<turtle_name>/cmd_vel` 持续发布速度命令，通过两阶段 P 控制器（先平移到目标点、再原地旋转到目标朝向）驱动海龟平滑运动，直到到达目标或超时。内部使用 `MultiThreadedExecutor`（2 线程）避免服务回调等待位姿时的死锁。
- **`turtlesim_go_to_client`**：命令行客户端，解析 `<x> <y> <theta> [turtle_name]` 参数后调用上述服务，打印结果并退出。

> **注意**：GoTo.srv 文件注释写的是"teleport"，但实际实现是平滑的速度控制，不是瞬移。

可调控制参数（通过 ROS 参数设置）：`k_linear`（默认 1.5）、`k_angular`（默认 6.0）、`max_linear`（默认 2.0 m/s）、`max_angular`（默认 6.0 rad/s）、`dist_tolerance`（默认 0.05）、`theta_tolerance`（默认 0.05 rad）、`control_rate_hz`（默认 30 Hz）、`overall_timeout_sec`（默认 10 s）。

---

## 运行命令

```bash
# source 环境（从仓库根目录）
cd /root/Ros2Learning
source ./ros2_ws/scripts/source.sh jazzy

# 终端 1：启动 turtlesim
ros2 run turtlesim turtlesim_node

# 终端 2：启动 GoTo 服务端
ros2 run ros2_learning_turtlesim_go_to turtlesim_go_to_server

# 终端 3：调用客户端驱动海龟到 (5.5, 5.5, 1.57)
ros2 run ros2_learning_turtlesim_go_to turtlesim_go_to_client 5.5 5.5 1.57 turtle1

# 也可以直接用 ros2 service call
ros2 service call /ros2_learning/turtlesim/go_to \
  ros2_learning_turtlesim_go_to/srv/GoTo \
  "{turtle_name: 'turtle1', x: 5.5, y: 5.5, theta: 1.57}"
```

---

## 输入/输出

### 提供的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/ros2_learning/turtlesim/go_to` | `ros2_learning_turtlesim_go_to/srv/GoTo` | 驱动指定海龟平滑运动到目标位姿 |

**GoTo 服务定义：**

```
# Request
string turtle_name   # 海龟名称（空则默认 "turtle1"）
float32 x
float32 y
float32 theta        # 目标朝向（弧度）
---
# Response
bool success
string message
```

### 订阅的话题（运行时动态创建）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/<turtle_name>/pose` | `turtlesim/msg/Pose` | 获取海龟当前位姿 |

### 发布的话题（运行时动态创建）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/<turtle_name>/cmd_vel` | `geometry_msgs/msg/Twist` | 向海龟发送速度命令 |

---

## 验收测试

暂无，待补充。

（可手动验收：客户端调用后终端应输出 `SUCCESS: ok: reached (5.5, 5.5, 1.57)`，turtlesim 窗口中海龟应平滑移动到目标位置并调整朝向。）

---

## 已知限制

- 服务为阻塞式（同步等待控制循环结束），单次请求最长阻塞时间由 `overall_timeout_sec` 决定（默认 10s）。
- 不支持并发请求：若服务端正在执行一个目标，新的请求将立即返回 `success: false, message: "server is busy"`。
- 客户端 `call_timeout_sec` 默认仅 2s，若目标较远超时会误报失败；控制实际运行到 `overall_timeout_sec`。实际使用时建议增大客户端超时或改用 action。
- 依赖 `turtlesim` 包，不适用于真实机器人场景。
- GoTo.srv 文件注释称为"teleport"与实现不符，属于历史遗留文档错误。

---

## Build

```bash
cd /root/Ros2Learning/ros2_ws
source ./scripts/source.sh jazzy
colcon build --symlink-install --packages-select ros2_learning_turtlesim_go_to
```
