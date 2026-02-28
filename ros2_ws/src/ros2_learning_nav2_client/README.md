# ros2_learning_nav2_client

> 参数化的 Nav2 `NavigateToPose` action 客户端，演示如何可靠地等待导航栈就绪、发布初始位姿并发送导航目标。

## 功能描述

本包实现了一个独立可执行的 Nav2 action 客户端节点 `nav2_client`（节点名 `nav2_minimal_client`），用于学习如何通过 rclcpp_action 与 Nav2 导航栈交互。

**完整的启动前置检查序列**：节点在发送导航目标之前会依序执行一组前置等待逻辑，避免因竞态条件导致目标被服务端拒绝：① 若启用仿真时间，等待 `/clock` 话题非零（最长 `clock_wait_timeout_sec` 秒）；② 等待 `navigate_to_pose` action server 上线（最长 10 s）；③ 通过 `/bt_navigator/get_state` 查询 Nav2 lifecycle 状态，若处于 INACTIVE 则尝试通过 lifecycle manager（`/lifecycle_manager_navigation/manage_nodes`）发送 STARTUP 命令，若 manager 不可用则直接调用 `/bt_navigator/change_state` 激活（最长 `nav2_active_timeout_sec` 秒）；④ 发布初始位姿到 `/initialpose`（连续发 5 次，间隔 200 ms）；⑤ 等待 `/amcl_pose` 话题返回一条消息，确认 AMCL 定位已就绪（最长 `amcl_pose_timeout_sec` 秒）；⑥ 等待 TF `map -> base_link` 变换可用（最长 `tf_wait_timeout_sec` 秒）。

**目标发送与结果处理**：调用 `async_send_goal` 异步发送目标，注册反馈回调打印 `distance_remaining`。异步等待结果最长 60 s，并针对 SUCCEEDED / ABORTED / CANCELED / TIMEOUT 四种情况分别记录日志。超时时自动调用 `async_cancel_goal` 取消当前目标。

**全参数化设计**：所有坐标系名称、初始位姿、目标位姿、超时时长、lifecycle 服务名等均通过 ROS 2 参数声明，可在运行时通过 `-p key:=value` 覆盖，无需修改代码。

学习要点：`rclcpp_action::Client` 的异步发送与结果等待模式、Nav2 lifecycle 状态机的查询与触发方式、`tf2_ros::Buffer` + `TransformListener` 的等待用法、`use_sim_time` 的正确处理（避免重复声明导致运行时异常）。

## 运行命令

```bash
# 1. 环境准备（每个终端均需执行）
source /opt/ros/jazzy/setup.bash
source /root/Ros2Learning/ros2_ws/install/setup.bash

# 2. 前提：需要先启动 Gazebo 仿真和 Nav2 导航栈
#    （参考 ros2_learning_task_runner 的启动流程）

# 3. 使用默认参数运行（目标点 x=1.0, y=0.0，仿真时间模式）
ros2 run ros2_learning_nav2_client nav2_client

# 4. 自定义目标位姿（真实时间模式）
ros2 run ros2_learning_nav2_client nav2_client \
  --ros-args \
  -p use_sim_time:=false \
  -p goal_x:=2.0 \
  -p goal_y:=1.5 \
  -p goal_yaw:=1.57

# 5. 自定义初始位姿 + 目标位姿（仿真时间模式，需 Nav2 已启动）
ros2 run ros2_learning_nav2_client nav2_client \
  --ros-args \
  -p use_sim_time:=true \
  -p initial_x:=0.0 \
  -p initial_y:=0.0 \
  -p initial_yaw:=0.0 \
  -p goal_x:=3.0 \
  -p goal_y:=-1.0 \
  -p goal_yaw:=0.0

# 6. 跳过 Nav2 active 等待（Nav2 已确认 active 时可加速启动）
ros2 run ros2_learning_nav2_client nav2_client \
  --ros-args \
  -p wait_nav2_active:=false \
  -p wait_amcl_pose:=false

# 7. 指定非默认的 lifecycle 服务名（例如自定义命名空间的 Nav2 栈）
ros2 run ros2_learning_nav2_client nav2_client \
  --ros-args \
  -p lifecycle_get_state_service:=/bt_navigator/get_state \
  -p lifecycle_manage_nodes_service:=/lifecycle_manager_navigation/manage_nodes
```

## 输入/输出

### 发布的话题

| 话题名 | 消息类型 | 说明 |
|---|---|---|
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 启动时连续发布 5 次初始位姿，供 AMCL 设置机器人初始定位 |

### 订阅的话题

| 话题名 | 消息类型 | 说明 |
|---|---|---|
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 等待 AMCL 输出定位结果，确认初始位姿已被接受 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间模式下等待时钟非零 |

### 调用的 Action

| Action 名 | 类型 | 说明 |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 发送导航目标，接收 `distance_remaining` 反馈，等待 SUCCEEDED/ABORTED/CANCELED 结果 |

### 调用的服务

| 服务名 | 类型 | 说明 |
|---|---|---|
| `/bt_navigator/get_state` | `lifecycle_msgs/srv/GetState` | 查询 bt_navigator 生命周期状态（默认，可通过参数修改） |
| `/bt_navigator/change_state` | `lifecycle_msgs/srv/ChangeState` | lifecycle manager 不可用时直接激活目标节点 |
| `/lifecycle_manager_navigation/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | 通过 lifecycle manager 一键 STARTUP 整个导航栈 |

### 节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `map_frame` | string | `"map"` | 地图坐标系名 |
| `base_frame` | string | `"base_link"` | 机器人基座坐标系名 |
| `initial_x` | double | `0.0` | 初始位姿 X（m） |
| `initial_y` | double | `0.0` | 初始位姿 Y（m） |
| `initial_yaw` | double | `0.0` | 初始位姿偏航角（rad） |
| `goal_x` | double | `1.0` | 目标位姿 X（m） |
| `goal_y` | double | `0.0` | 目标位姿 Y（m） |
| `goal_yaw` | double | `0.0` | 目标位姿偏航角（rad） |
| `use_sim_time` | bool | `true` | 是否使用仿真时间 |
| `wait_nav2_active` | bool | `true` | 是否等待 Nav2 进入 ACTIVE 后再发目标 |
| `auto_startup_nav2` | bool | `true` | 检测到 INACTIVE 时是否自动尝试启动 Nav2 |
| `wait_amcl_pose` | bool | `true` | 是否等待 AMCL 位姿到达后再发目标 |
| `tf_wait_timeout_sec` | double | `10.0` | TF 等待超时（s） |
| `clock_wait_timeout_sec` | double | `20.0` | 仿真时钟等待超时（s） |
| `nav2_active_timeout_sec` | double | `20.0` | Nav2 active 等待超时（s） |
| `amcl_pose_timeout_sec` | double | `10.0` | AMCL 位姿等待超时（s） |
| `lifecycle_get_state_service` | string | `"/bt_navigator/get_state"` | 用于查询 Nav2 状态的 lifecycle 服务名 |
| `lifecycle_manage_nodes_service` | string | `"/lifecycle_manager_navigation/manage_nodes"` | lifecycle manager 服务名 |
| `amcl_pose_topic` | string | `"/amcl_pose"` | AMCL 位姿话题名 |

## 验收测试

暂无，待补充。

## 已知限制

- **必须先启动 Nav2**：本节点不会启动导航栈，若 `navigate_to_pose` action server 在 10 s 内未出现，节点直接退出。需预先启动 Gazebo 仿真 + Nav2（含 `bt_navigator`、`amcl`、`lifecycle_manager_navigation` 等节点）。
- **仿真时间默认开启**：`use_sim_time` 默认为 `true`，若在真实机器人或无 Gazebo 的环境运行，必须通过 `-p use_sim_time:=false` 关闭，否则节点会因等待 `/clock` 超时而提前退出。
- **单次导航设计**：节点运行一次只发送一个目标，结果返回（或超时）后即退出，不是持续运行的导航服务。如需循环导航，请参考 `ros2_learning_task_runner`。
- **`use_sim_time` 重复声明保护**：rclcpp 内部的 `TimeSource` 可能已经声明 `use_sim_time` 参数，代码中已做 `has_parameter` 检查；如果在使用 `rclcpp::NodeOptions` 传入该参数时仍然遇到"parameter already declared"异常，需检查节点构造顺序。
- **导航超时固定为 60 s**：`async_get_result` 的等待时长在代码中硬编码为 60 s，对于长距离导航任务可能不足，当前无法通过参数覆盖。
- 依赖：`nav2_msgs`、`lifecycle_msgs`、`tf2`、`tf2_geometry_msgs`、`rclcpp_action`。需安装 `ros-jazzy-nav2-msgs` 及完整 Nav2 栈（`ros-jazzy-navigation2`）。
