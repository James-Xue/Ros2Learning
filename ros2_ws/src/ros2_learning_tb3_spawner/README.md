# ros2_learning_tb3_spawner

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
"}