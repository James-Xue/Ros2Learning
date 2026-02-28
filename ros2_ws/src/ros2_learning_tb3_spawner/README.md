# ros2_learning_tb3_spawner

> 通过自定义 ROS 2 服务 `/spawn_tb3` 在已运行的 Gazebo 世界中动态生成额外的 TurtleBot3，并自动对 SDF 话题加前缀以支持多机器人并存。

在运行 `turtlebot3_gazebo` 世界后，通过 **ROS 2 Service** 在指定位置再生成一台 TurtleBot3。

## 前提

已启动 Gazebo 世界（示例）：

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

并构建/叠加本仓库工作空间：

```bash
cd Ros2Learning/ros2_ws
source ./scripts/source.sh jazzy
colcon build --symlink-install
source ./install/setup.bash
```

## 启动 spawner 节点

```bash
ros2 run ros2_learning_tb3_spawner tb3_spawner
```

## 一键启动（世界 + 再生成一台机器人 + bridge）

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch ros2_learning_tb3_spawner tb3_two_robots.launch.py model:=waffle_pi second_name:=tb3_2 second_x:=1.0 second_y:=0.0 second_yaw:=0.0
```

该 launch 会：
- 启动 `turtlebot3_gazebo/turtlebot3_world.launch.py`（世界 + 第一台机器人）
- 生成第二台机器人（对 SDF 内部 topic 做了 `/<name>/...` 前缀，避免多机器人 topic 冲突）
- 启动第二台机器人的 `ros_gz_bridge`（并按模型类型选择性启动 `ros_gz_image`）

控制第二台机器人时，注意使用带 namespace 的话题，例如：

```bash
ros2 topic pub -r 10 /tb3_2/cmd_vel geometry_msgs/msg/TwistStamped \
	"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

## 调用服务生成第二台机器人

```bash
ros2 service call /spawn_tb3 ros2_learning_tb3_spawner/srv/SpawnTb3 "{name: 'tb3_2', model: 'waffle_pi', x: 1.0, y: 0.0, z: 0.01, yaw: 0.0}"
```

说明：
- `model` 支持 `burger|waffle|waffle_pi`，默认会优先读取环境变量 `TURTLEBOT3_MODEL`。
- 该实现通过 Gazebo 的 `/spawn_entity` 生成 `turtlebot3_gazebo/models/turtlebot3_<model>/model.sdf`。

---

## 功能描述

`tb3_spawner` 节点对外暴露 `/spawn_tb3` 服务（类型 `ros2_learning_tb3_spawner/srv/SpawnTb3`）。接收到请求后：

1. 从 `turtlebot3_gazebo` 包中读取对应型号的 `model.sdf`。
2. 若实体名与模型名不同（即第二台机器人），自动对 SDF 内的所有 Gazebo transport 话题（`cmd_vel`、`odom`、`scan`、`imu`、`joint_states`、`tf`、`camera/*`）加上 `/<name>/` 前缀，避免多机器人话题冲突。
3. 调用 Gazebo 内部的 `/spawn_entity` 服务完成实体注入。

节点内部使用 `MultiThreadedExecutor`（2 线程），防止同步等待 Gazebo 响应时服务回调阻塞自身 executor 的死锁问题。

`tb3_two_robots.launch.py` 是一键式双机器人启动文件，无需手动调用服务；`tb3_spawner.launch.py` 则仅启动服务节点供手动调用。

---

## 运行命令

```bash
# source 环境（从仓库根目录）
cd /root/Ros2Learning
source ./ros2_ws/scripts/source.sh jazzy

# 前提：先启动 Gazebo 世界（在另一个终端）
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 方式一：一键启动（世界 + 第二台机器人 + bridge）
ros2 launch ros2_learning_tb3_spawner tb3_two_robots.launch.py \
  model:=waffle_pi second_name:=tb3_2 second_x:=1.0 second_y:=0.0 second_yaw:=0.0

# 方式二：仅启动 spawner 节点，再手动调用服务
ros2 run ros2_learning_tb3_spawner tb3_spawner

ros2 service call /spawn_tb3 ros2_learning_tb3_spawner/srv/SpawnTb3 \
  "{name: 'tb3_2', model: 'waffle_pi', x: 1.0, y: 0.0, z: 0.01, yaw: 0.0}"

# 控制第二台机器人（带 namespace 的话题）
ros2 topic pub -r 10 /tb3_2/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

---

## 输入/输出

### 提供的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/spawn_tb3` | `ros2_learning_tb3_spawner/srv/SpawnTb3` | 在 Gazebo 中生成指定型号的 TurtleBot3 |

**SpawnTb3 服务定义：**

```
# Request
string name    # 实体名称（空则自动生成唯一名）
string model   # 型号：burger / waffle / waffle_pi（空则读 TURTLEBOT3_MODEL 环境变量）
float64 x
float64 y
float64 z
float64 yaw
---
# Response
bool success
string message
```

### 依赖的服务（客户端调用）

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/spawn_entity` | `ros_gz_interfaces/srv/SpawnEntity` | Gazebo 内部实体生成服务 |

### 第二台机器人生成后的话题（以 `tb3_2` 为例）

| 话题 | 类型 | 方向 |
|------|------|------|
| `/tb3_2/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS -> Gazebo |
| `/tb3_2/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS |
| `/tb3_2/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS |
| `/tb3_2/imu` | `sensor_msgs/msg/Imu` | Gazebo -> ROS |
| `/tb3_2/joint_states` | `sensor_msgs/msg/JointState` | Gazebo -> ROS |
| `/tb3_2/tf` | `tf2_msgs/msg/TFMessage` | Gazebo -> ROS |
| `/tb3_2/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo -> ROS（仅 waffle/waffle_pi）|
| `/tb3_2/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo -> ROS（仅 waffle/waffle_pi）|

---

## 验收测试

暂无，待补充。

（可手动验收：调用 `/spawn_tb3` 服务后，`ros2 topic list | grep tb3_2` 应出现对应话题；Gazebo 中应看到第二台机器人出现在指定坐标位置。）

---

## 已知限制

- 依赖 `turtlebot3_gazebo` 包提供 SDF 模型文件，若未安装该包则服务调用失败。
- SDF 话题前缀通过字符串替换实现，仅覆盖已知的固定话题名；若 `turtlebot3_gazebo` 上游更新 SDF 内的话题名则需同步更新 `patch_sdf_topics()`。
- 不支持删除已生成的实体（无 despawn 服务）。
- `tb3_two_robots.launch.py` 生成的 bridge YAML 写入系统临时目录（`/tmp/`），重启后文件会消失，但节点生命周期内不受影响。
- 第一台机器人（由 `turtlebot3_gazebo` world launch 生成）的话题不带 namespace，与第二台不对称，使用时需注意区分。