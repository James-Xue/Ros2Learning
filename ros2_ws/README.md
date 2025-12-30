TurtleBot3 Task-Driven Navigation System (ROS 2)
1. 项目简介

本项目基于 ROS 2 Jazzy 与 TurtleBot3（Waffle Pi）仿真模型，实现了一套任务驱动（Task-driven）的移动机器人导航系统。

与官方示例不同，本项目不以“跑通 demo”为目标，而是从工程角度对导航系统进行组织与封装，实现：

一键启动的机器人系统（bringup）

基于任务接口的自动导航

对导航任务的成功 / 失败 / 超时进行明确管理

为后续 机械臂抓取与移动操作（Mobile Manipulation） 预留系统接口

该项目作为个人学习与工程实践项目，重点体现 ROS2 系统集成能力、工程组织能力和任务层设计能力。

2. 项目目标与阶段规划
阶段目标（当前阶段：里程碑 1）

里程碑 1：任务驱动的自动导航系统

在 Gazebo 仿真环境中，通过自定义任务节点指定目标位姿，
机器人能够可靠地自主导航至指定终点，并对异常情况进行处理。

后续规划（不在当前里程碑范围内）

里程碑 2：多目标点任务与任务状态机

里程碑 3：机械臂（MoveIt2）抓取仿真

里程碑 4：移动机器人 + 机械臂协同任务（Mobile Manipulation）

3. 系统架构概览
软件分层结构
```text
+----------------------------------+
| Mission Layer (Custom)           |
| - 任务接口                       |
| - 超时 / 失败处理                |
| - 状态管理与日志                 |
+----------------------------------+
| Navigation Layer (Nav2)          |
| - 路径规划                       |
| - 控制与避障                     |
| - 行为树                         |
+----------------------------------+
| Localization / Mapping           |
| - SLAM Toolbox / AMCL            |
+----------------------------------+
| Robot Abstraction                |
| - TF                             |
| - cmd_vel / scan                 |
+----------------------------------+
| Simulation (Gazebo)              |
+----------------------------------+
```


本项目的核心代码主要集中在 Mission Layer 与 System Bringup，底层算法由成熟的 ROS2 官方组件提供。

4. 项目结构
```text
ros2_ws/
└── src/
    ├── bringup/                  # 系统启动与模式管理（自定义）
    │   ├── launch/
    │   │   ├── sim.launch.py
    │   │   ├── slam.launch.py
    │   │   └── nav.launch.py
    │   ├── config/
    │   │   ├── nav2_params.yaml
    │   │   ├── amcl.yaml
    │   │   └── robot.yaml
    │   └── maps/
    │       └── office.yaml
    │
    ├── mission_manager/           # 任务层（自定义核心）
    │   ├── mission_manager/
    │   │   └── go_to_pose.py
    │   └── package.xml
    │
    ├── robot_description/         # 机器人模型（URDF/Xacro）
    └── sim_gazebo/                # Gazebo world 与仿真资源
```

5. 环境依赖

OS：Ubuntu 24.04（WSL2/vmware）

ROS 2：Jazzy

仿真：Gazebo (GZ Sim / Harmonic)

机器人模型：TurtleBot3 Waffle Pi

导航：Nav2

建图/定位：slam_toolbox / AMCL

6. 启动方式
6.1 启动仿真与导航系统
ros2 launch bringup sim.launch.py mode:=nav


启动内容包括：

Gazebo 仿真环境

TurtleBot3 模型

TF 与传感器

Nav2 全套导航栈

RViz 可视化

6.2 通过任务接口执行导航（核心）

通过自定义任务节点指定终点，而非使用 RViz 手动点目标：

ros2 run mission_manager go_to_pose \
  --x 2.0 \
  --y 1.5 \
  --yaw 1.57 \
  --timeout 120

行为说明

使用 NavigateToPose Action 向 Nav2 发送导航目标

等待导航结果

根据结果返回：

成功（Reached Goal）

超时（Timeout）

失败（Aborted / Canceled）

7. 里程碑 1 验收标准

当前阶段完成情况以工程可复现性与系统稳定性为准：

 系统可通过一条 launch 命令启动

 可通过自定义任务接口指定导航终点

 机器人可自主规划路径并避障

 导航任务支持超时与失败处理

 连续多次导航成功率 ≥ 80%

 README 与参数配置清晰可复现

8. 工程设计要点

不修改 Nav2 内部代码，专注系统集成与任务层设计

所有运行模式（建图 / 定位 / 导航）通过 bringup 统一管理

Mission 层与底层导航解耦，便于未来扩展机械臂任务

参数与启动逻辑集中管理，避免“手动跑 demo 式流程”

9. 后续扩展方向

多任务序列（Waypoints / Patrol）

行为树或状态机（BT / FSM）

MoveIt2 机械臂抓取仿真

移动抓取任务（Mobile Manipulation）

从仿真迁移至真实硬件

10. 项目定位说明

本项目关注 ROS2 机器人系统的工程实现与任务层设计，
而非重复实现已有导航或 SLAM 算法。
项目重点体现：

ROS2 系统集成能力

任务驱动架构设计

工程化启动、参数与异常管理能力
