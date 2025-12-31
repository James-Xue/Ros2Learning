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

## Nav2 C++ 代码客户端最小示例（`nav2_client`）
> 目标：用 C++ action client 发送单个导航目标，包含初始位姿发布、反馈打印与超时取消，适配仿真时间。

代码位置：`ros2_ws/src/ros2_learning_cpp/src/nav2_client.cpp`  
构建配置：`ros2_ws/src/ros2_learning_cpp/CMakeLists.txt`（可执行名 `nav2_client`）

### 依赖（已在 package.xml / CMakeLists.txt 声明）
- `rclcpp`, `rclcpp_action`
- `nav2_msgs`
- `geometry_msgs`
- `tf2`, `tf2_geometry_msgs`

### 运行前准备
1. 启动仿真与 Nav2（与上文一致），确保全部使用 `use_sim_time:=true`。
2. Source 环境（示例）：
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd /workspace/Ros2Learning/ros2_ws
   colcon build --packages-select ros2_learning_cpp
   source install/setup.bash
   ```
   > 若本地未安装 `colcon`，请先安装：`sudo apt install python3-colcon-common-extensions`。

### 运行命令
```bash
ros2 run ros2_learning_cpp nav2_client
```

## VS Code 一键启动 + 断点调试（推荐）

仓库已提供一条“一键启动仿真 + Nav2 + RViz + 断点调试 nav2_client”的 VS Code 配置：

1. 在 VS Code 中打开本仓库，确保已拉取并执行过（用于步入 rclcpp 源码，可选）：
  ```bash
  ./ros2_ws/scripts/setup_rclcpp_source.sh jazzy
  ```
2. 在 VS Code 的 Run and Debug 面板选择：`ROS2: Debug nav2_client (Debug build)`，按 F5。
3. 该配置会自动：Debug 构建工作空间、启动 Gazebo/ Nav2/ RViz，并在 `Nav2Client::run`、`wait_for_server`、`publish_initial_pose`、`send_goal` 处预置断点。

### 节点行为说明
1. 设置 `use_sim_time` 参数，等待仿真时钟有效。
2. 向 `/initialpose` 连续发布 5 次初始位姿（frame_id=map），避免未定位直接导航。
3. 创建 `NavigateToPose` action client，等待服务器。
4. 发送目标点（默认 x=1.0, y=0.0, yaw=0），打印剩余距离反馈。
5. 60 秒内未完成则取消任务；成功/取消/异常均在日志提示。

### 快速验证与调试
- 观察终端日志是否出现 “Goal accepted” 和反馈距离。
- 结果状态若为 aborted/canceled，检查：
  - `ros2 node list | grep -E 'amcl|map_server|planner|controller'`
  - `ros2 run tf2_ros tf2_echo map base_link`
- 目标点可根据地图实际可行区域修改：编辑 `nav2_client.cpp` 中的目标坐标后重新构建。
