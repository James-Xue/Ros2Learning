# Ros2Learning

A ROS 2 simulation project for an autonomous vacuum robot.

## Quickstart

> 前提：已安装对应 ROS 2（如 `/opt/ros/jazzy` 存在）。

```bash
cd Ros2Learning/ros2_ws
./scripts/setup_ubuntu.sh jazzy   # 可选
./scripts/build.sh jazzy
source ./scripts/source.sh jazzy
```

## 仿真优先的总体设计（Gazebo + Jazzy）

> 说明：当前阶段以仿真为主，硬件暂不接入。方案成熟后再进入实现阶段。
> 目标读者：未接触过 ROS 的新手，但已了解话题、服务等基础概念。

### 架构示意（方向性）
```mermaid
flowchart LR
  subgraph Sim[Gazebo Sim]
    world[World/Robot Model]
    sensors[Sim Sensors]
  end
  subgraph ROS[ROS 2 System]
    desc[robot_description]
    ctrl[robot_control]
    bridge[ros_gz_bridge]
    mapping[robot_mapping]
    nav[robot_navigation]
    apps[robot_teleop/demo]
  end
  world --> sensors
  sensors --> bridge
  desc --> ctrl
  bridge --> mapping
  bridge --> nav
  ctrl --> bridge
  apps --> nav
```

### 总体架构（分层）
- 硬件抽象层：底盘与传感器接口（仿真中由 Gazebo 提供）。
- 感知层：里程计融合、障碍物感知、代价地图。
- 定位与建图：`slam_toolbox`（学习阶段优先）。
- 导航与决策：`nav2` 行为树与路径规划。
- 应用层：遥控、巡航、定点导航等学习示例。

### 包划分（多包，colcon 管理）
- `robot_description`：URDF/Xacro、TF 树、传感器定义。
- `robot_gz`：Gazebo 世界与模型资源、仿真启动、桥接配置。
- `robot_control`：`ros2_control` 与 diff drive 控制。
- `robot_bringup`：一键启动（仿真 + 机器人 + RViz）。
- `robot_mapping`：`slam_toolbox` 参数与 launch。
- `robot_navigation`：`nav2` 参数与行为树。
- `robot_teleop`：键盘/手柄遥控。

### 关键话题与 TF（约定）
- 话题：`cmd_vel`、`odom`、`scan`、`imu`、`map`、`tf`
- TF：`map -> odom -> base_link -> base_laser/base_imu/base_camera`

### 仿真验证的最小里程碑
1) Gazebo 中小车模型可加载
2) `cmd_vel` 控制底盘运动
3) `scan`/`imu`/`odom` 数据正常发布
4) RViz 中可视化 TF 与传感器数据

### 下一步优化方向（方案先行）
- 明确 Gazebo Sim（Harmonic）与 `ros_gz` 的桥接话题映射
- 统一参数目录与命名规范（`config/*.yaml`）
- 设计可复用的 launch 分层（仿真/建图/导航）

## 方案里程碑（细化）与验收标准

> 时间为单人估算（学习型项目，按自然日计）。

### M0：环境与基础概念熟悉（1-2 天）
- 工作内容：安装 ROS 2 Jazzy 与 Gazebo Sim（Harmonic）；了解工作空间、包与节点的基本概念；能运行官方 demo。
- 验收标准：`ros2` 与 `gz` 命令可用；`ros2 run demo_nodes_cpp talker` 与 `listener` 可互通。

### M1：机器人描述与仿真模型（2-3 天）
- 工作内容：完成小车 URDF/Xacro、基础 TF；理解 `robot_state_publisher` 与 TF 树。
- 验收标准：Gazebo 中可加载模型；`tf` 中存在 `base_link` 与 `base_laser` 等传感器帧。

### M2：底盘控制与话题闭环（2-3 天）
- 工作内容：接入 `ros2_control` diff drive；理解 `cmd_vel` 与 `odom` 语义。
- 验收标准：`ros2 topic pub /cmd_vel ...` 可驱动模型移动；`/odom` 持续发布且数值合理。

### M3：传感器模拟与桥接（2-3 天）
- 工作内容：配置 Gazebo 传感器插件；完成 `ros_gz_bridge` 映射；理解消息类型。
- 验收标准：`/scan`、`/imu`、`/camera`（如启用）在 ROS 侧稳定发布并可在 RViz 显示。

### M4：建图与可视化（2-3 天）
- 工作内容：集成 `slam_toolbox` 与 RViz 配置；理解 `map`、`odom`、`base_link` 的关系。
- 验收标准：可在 RViz 中实时生成 `map` 并保存；TF 树无断链与明显跳变。

### M5：导航与示例任务（3-5 天）
- 工作内容：集成 `nav2` 参数与行为树；学习代价地图与局部规划器。
- 验收标准：给定目标点后小车可规划与到达；路径平滑、无明显抖动或频繁重规划。

### 开发流程（建议）
```mermaid
flowchart TD
  A[明确需求/里程碑] --> B[设计包划分与话题/TF]
  B --> C[实现最小可跑版本]
  C --> D[验证与记录问题]
  D --> E[参数调优与重构]
  E --> F[添加功能与示例]
```

## 最低测试流程（手工）

> 说明：占位命令需替换为实际 launch 名称。

1) 启动仿真与系统  
   - 示例：`ros2 launch robot_bringup sim.launch.py`  
   - 验收：Gazebo 打开且小车可见；终端无致命报错。

2) 节点存活检查  
   - 命令：`ros2 node list`  
   - 验收：关键节点存在（控制、桥接、状态发布等）。

3) 话题发布检查  
   - 命令：`ros2 topic list`、`ros2 topic info /cmd_vel`、`ros2 topic info /odom`、`ros2 topic info /scan`  
   - 验收：关键话题存在，类型正确，且有发布者。

4) 话题频率与数据检查  
   - 命令：`ros2 topic hz /odom`、`ros2 topic hz /scan`、`ros2 topic echo /odom --once`  
   - 验收：`/odom`、`/scan` 有稳定频率；`/odom` 数据非零且合理。

5) TF 连通性检查  
   - 命令：`ros2 run tf2_ros tf2_echo map base_link` 或 `ros2 run tf2_tools view_frames`  
   - 验收：`map -> odom -> base_link` 连通无报错。

6) 控制闭环验证  
   - 命令：`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10`  
   - 验收：模型在 Gazebo 中移动，`/odom` 变化连续。
