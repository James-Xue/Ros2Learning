# Nav2 代码驱动学习计划（基于 TurtleBot3 仿真）

> 目标：从“在 RViz 点导航”过渡到“用代码控制 Nav2”。默认环境为 ROS 2 Jazzy + TurtleBot3 waffle_pi + Gazebo 仿真。

## 前提环境
- 已能启动仿真：`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
- 已能启动 Nav2：`ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=...</map.yaml`
- 已在 RViz 设置初始位姿（或代码中调用 setInitialPose）

## 阶段 1：熟悉 Nav2 Action 接口
**目标**：会用代码发送/取消导航目标，获取反馈与结果。  
**建议**：先用 Python `nav2_simple_commander`（`BasicNavigator`），再看 C++ `rclcpp_action`。

步骤：
1. 创建自定义脚本（例如 `nav2_client.py`），初始化 `BasicNavigator`，设置 `use_sim_time`。
2. 用 `setInitialPose` 设置初始位姿。
3. 调用 `goToPose` 发送单个目标，打印反馈/结果。
4. 增加取消逻辑：超时或用户输入时调用 `cancelTask`。

## 阶段 2：多点巡航与失败重试
**目标**：让机器人按多个 Waypoint 依次巡航，支持超时/重试。  
步骤：
1. 定义 Waypoint 列表（x/y/yaw）。  
2. 依次调用 `goToPose`（或 `followWaypoints`）。  
3. 为每个目标设置超时与重试次数，记录结果汇总。  
4. 打印或发布执行报告，便于调试。

## 阶段 3：外部事件驱动的目标生成
**目标**：从外部输入生成导航目标，模拟“任务层”控制。  
可选输入源：JSON/CSV 文件、键盘输入、自定义 ROS 2 topic、REST/MQTT 网关。  
步骤：
1. 解析输入并转换为 `PoseStamped`。  
2. 简单校验（坐标范围、是否落在地图占用区外）。  
3. 发送导航；异常时记录并可重试或跳过。

## 阶段 4：健康检查与可视化输出
**目标**：提升稳健性和可观测性。  
步骤：
1. 启动前检查关键节点：`amcl`、`map_server`、`controller_server`、`planner_server`。  
2. TF 检查：使用 `tf2_ros.Buffer` 查询 `map -> base_link`。  
3. 将当前目标、状态、重试次数发布到自定义 topic，或打印在终端；需要时在 RViz 显示。

## 验证与调试要点
- 确保所有节点使用 `use_sim_time:=true`，与仿真时钟一致。  
- 先设初始位姿，再发送目标；或在代码中调用 `setInitialPose`。  
- 常用命令：  
  - `ros2 run tf2_ros tf2_echo map base_link`（验证 TF）  
  - `ros2 topic echo /map --once`（验证地图）  
  - `ros2 node list | grep -E 'amcl|map_server|planner|controller'`（验证节点）  
- 机器人不动/频繁重规划：检查 `/scan` 数据、costmap 膨胀、目标点是否在可行区域。  
- 导航过程中避免手动发布 `cmd_vel`，以免与控制器冲突。
