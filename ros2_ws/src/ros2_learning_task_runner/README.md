# ros2_learning_task_runner

> 基于 Nav2 动作接口的多点取放任务编排节点，演示如何将导航、服务调用、状态发布串联成完整的机器人工作流。

## 功能描述

本包实现一个固定流程的"取放"（Pick-and-Place）任务编排节点。节点启动后，按顺序依次前往 YAML 配置文件中定义的每个采集点（Pickup），在每个点分别调用机械臂抓取服务（`/manipulation/pick`），再导航到统一的投放点（Dropoff），调用放置服务（`/manipulation/place`），然后继续处理下一个采集点，直到列表耗尽。

节点的核心学习点在于 **ROS 2 通信机制的综合运用**：使用 `rclcpp_action` 异步调用 Nav2 的 `navigate_to_pose` 动作（长时任务，有实时反馈），使用 `rclcpp::Client<Trigger>` 同步调用抓取/放置服务（短时请求-响应），并通过 Publisher 对外广播任务状态和剩余距离，方便接入监控 UI 或调试工具。

节点还演示了仿真环境下的常见工程细节：启动前依次等待仿真时钟（`/clock`）就绪、TF 坐标链路（`map` → `base_link`）可查、Nav2 动作服务器上线、以及机械臂占位服务可达；可选地向 `/initialpose` 发布初始位姿以触发 AMCL 重定位。任务配置（点位坐标）完全外置于 YAML 文件，无需重新编译即可调整路径。

本包同时定义了自定义消息类型 `TaskStatus`，包含当前状态字符串、阶段名、已完成/总采集点数量以及最近一次错误信息，方便下游节点订阅并展示进度。

## 运行命令

```bash
# 1. source 环境
source /opt/ros/jazzy/setup.bash
source /root/Ros2Learning/ros2_ws/install/setup.bash

# 2. 前置条件：先在其他终端启动 Gazebo + Nav2（略）

# 3. 使用默认配置启动任务（同时会启动 manipulation_stub 占位服务）
ros2 launch ros2_learning_task_runner task_runner_sim.launch.py

# 4. 覆盖参数示例：使用自定义任务计划、关闭仿真时钟
ros2 launch ros2_learning_task_runner task_runner_sim.launch.py \
  task_config:=/path/to/my_plan.yaml \
  use_sim_time:=false

# 5. 覆盖参数示例：让节点自动发布初始位姿（帮助 AMCL 定位）
ros2 launch ros2_learning_task_runner task_runner_sim.launch.py \
  publish_initial_pose:=true \
  initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0

# 6. 实时观察任务状态
ros2 topic echo /task_status

# 7. 实时观察剩余距离（导航过程中）
ros2 topic echo /distance_remaining
```

## 输入/输出

### 发布的话题（Publisher）

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/task_status` | `ros2_learning_task_runner/msg/TaskStatus` | 当前任务阶段、状态字符串、采集点进度、最近错误 |
| `/distance_remaining` | `std_msgs/msg/Float32` | Nav2 反馈的到目标点剩余距离（米），导航过程中实时更新 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 启动时可选发布，用于触发 AMCL 重定位 |

### 调用的动作（Action Client）

| 动作名 | 类型 | 说明 |
|---|---|---|
| `navigate_to_pose`（可通过参数 `nav2_action_name` 修改） | `nav2_msgs/action/NavigateToPose` | 向 Nav2 发送导航目标，异步等待结果，超时后主动取消 |

### 调用的服务（Service Client）

| 服务名 | 类型 | 说明 |
|---|---|---|
| `/manipulation/pick`（可通过参数 `pick_service` 修改） | `std_srvs/srv/Trigger` | 到达采集点后触发抓取动作 |
| `/manipulation/place`（可通过参数 `place_service` 修改） | `std_srvs/srv/Trigger` | 到达投放点后触发放置动作 |

### 自定义消息 `TaskStatus.msg`

```
string state          # 当前状态（空闲/前往采集点/执行抓取/前往投放点/执行放置/任务失败）
string phase          # 当前细分阶段名称
uint32 pickup_index   # 当前处理中的采集点编号（从 1 开始）
uint32 pickup_total   # 采集点总数
string last_error     # 最近一次错误描述（无错误时为空）
```

### 任务配置文件格式（`config/task_plan.yaml`）

```yaml
map_frame: map          # 地图坐标系（可省略，默认 map）

dropoff:                # 统一投放点（必填）
  x: 0.0
  y: 0.0
  yaw: 0.0              # 偏航角，弧度制

pickups:                # 采集点列表（顺序执行，至少1个）
  - x: 1.0
    y: 0.0
    yaw: 0.0
  - x: 1.0
    y: 1.0
    yaw: 1.57
```

### 可配置的节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `task_config` | string | （包内 config/task_plan.yaml） | 任务 YAML 文件绝对路径 |
| `use_sim_time` | bool | `true` | 是否使用仿真时钟 |
| `map_frame` | string | `map` | 地图 TF 坐标系 |
| `base_frame` | string | `base_link` | 机器人底盘 TF 坐标系 |
| `nav2_action_name` | string | `navigate_to_pose` | Nav2 动作服务器名称 |
| `pick_service` | string | `/manipulation/pick` | 抓取服务名称 |
| `place_service` | string | `/manipulation/place` | 放置服务名称 |
| `clock_wait_timeout_sec` | double | `20.0` | 等待仿真时钟超时（秒） |
| `tf_wait_timeout_sec` | double | `10.0` | 等待 TF 链路超时（秒） |
| `navigation_timeout_sec` | double | `120.0` | 单次导航超时（秒） |
| `publish_initial_pose` | bool | `false` | 是否启动时发布初始位姿 |
| `initial_x/y/yaw` | double | `0.0` | 初始位姿坐标和偏航角 |

## 验收测试

暂无，待补充。

后续可添加的测试方向：
- 单元测试 `load_task_config()` 对合法/非法 YAML 的解析行为
- 单元测试 `make_pose()` 偏航角到四元数的转换正确性
- 集成测试：使用 mock Action Server + mock Service 验证完整任务循环

```bash
# 当测试存在后，运行方式：
cd /root/Ros2Learning/ros2_ws
colcon test --packages-select ros2_learning_task_runner
colcon test-result --verbose
```

## 已知限制

- **强依赖 Nav2 运行时**：节点启动后会阻塞等待 `navigate_to_pose` 动作服务器（最多 10 秒），若 Nav2 未运行则直接以 `kFailed` 状态退出。
- **强依赖 manipulation_stub**：`task_runner_sim.launch.py` 会一并启动 `ros2_learning_manipulation_stub` 包中的占位节点，该包必须已编译安装。
- **必须使用仿真时间**：默认 `use_sim_time:=true`，若在实体机器人上使用需手动设置为 `false`，否则节点会在等待 `/clock` 阶段超时退出。
- **任务失败不中止，而是跳过**：每个采集点的导航/抓取/放置任意一步失败后，当前点会被 `continue` 跳过，继续执行下一个点。这对于学习调试足够，但不适用于需要严格保证幂等性的生产场景。
- **线性单线程逻辑**：`run()` 是阻塞式串行循环，不支持并发执行多个导航目标或中途接收外部停止指令。
- **仅支持平面 2D 导航**：位姿结构 `Pose2D` 只包含 `x/y/yaw`，不支持三维空间或坡道等场景。
