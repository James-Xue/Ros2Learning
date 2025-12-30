# TurtleBot3 + Gazebo + Nav2 + RViz2 快速导航教程

> 适用场景：在 Gazebo 中运行 TurtleBot3（waffle_pi）仿真，使用官方 `turtlebot3_navigation2` 包，通过 RViz2 发送导航目标完成简单导航实验。

## 环境准备
- ROS 2 Jazzy（假设 `/opt/ros/jazzy` 已安装）
- Gazebo Sim（Harmonic，随 TurtleBot3 仿真包）
- TurtleBot3 模型：`waffle_pi`

在所有终端执行：
```bash
export TURTLEBOT3_MODEL=waffle_pi
source /opt/ros/jazzy/setup.bash
```

## 启动顺序
1) **终端 A：启动仿真**
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2) **终端 B：启动 Nav2（仿真时间 + 预置地图）**
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
     use_sim_time:=true \
     map:=/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml
   ```

## RViz2 操作要点
1. **Fixed Frame** 选择 `map`（若暂时缺失可先继续设置初始位姿）。
2. 点击 **“2D Pose Estimate”**，在地图上给机器人一个大致初始位姿；AMCL 会据此开始发布 `map -> odom`。
3. 点击 **“2D Goal Pose”**，在地图上选择目标位置和朝向，即可触发导航。
4. 观察 Nav2 面板与终端日志：应看到路径规划、控制输出，Gazebo 中机器人会移动。

## 必要检查命令
- 节点与话题：
  ```bash
  ros2 node list | grep -E 'amcl|map_server|planner|controller'
  ros2 topic echo /map --once
  ```
- TF 连通性：
  ```bash
  ros2 run tf2_ros tf2_echo map base_link
  ```

## 常见问题与快速解法
- **RViz 提示没有 map frame**：AMCL 未发布 `map->odom`。先用 “2D Pose Estimate” 设置初始位姿，确认 `tf2_echo map base_link` 正常输出。
- **机器人不动或频繁重规划**：检查 `/scan` 是否有数据、`use_sim_time` 是否在所有节点一致、目标点是否在可行区域。
- **地图未加载**：确认 `map:=.../map.yaml` 路径存在且 `image:` 可访问；`ros2 topic echo /map --once` 应有输出。

## 最小验证流程（推荐顺序）
1. Gazebo 中模型出现且无报错。
2. `ros2 topic echo /map --once` 成功。
3. `ros2 run tf2_ros tf2_echo map base_link` 有数据。
4. RViz 设初始位姿后，发送 2D Goal Pose，机器人成功到达目标。

## 小贴士
- 导航过程中不要同时手动发布速度指令，以免与控制器冲突。
- 可在 RViz “File -> Save Config” 保存当前可视化配置，方便下次直接加载。
