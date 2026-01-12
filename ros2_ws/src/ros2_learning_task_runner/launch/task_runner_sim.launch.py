"""Task runner + manipulation stub 启动脚本（仿真模式）

此文件用于在仿真环境下启动任务编排与抓取/放置占位服务。

功能概述：
- 声明可由命令行覆盖的启动参数（`task_config`, `use_sim_time`）。
- 解析包内默认的任务配置文件路径（`config/task_plan.yaml`）。
- 启动两个 ROS 2 节点：`task_runner`（任务编排）和 `manipulation_stub`（抓取/放置占位）。

使用方法示例（在已经 source 了工作空间与 ROS 环境后）：
```bash
ros2 launch ros2_learning_task_runner task_runner_sim.launch.py
# 或者覆盖参数：
ros2 launch ros2_learning_task_runner task_runner_sim.launch.py task_config:=/path/to/my_plan.yaml use_sim_time:=false
```

Imports 说明：
- `LaunchDescription`：构建并返回 launch 的顶层描述对象，包含参数声明与要执行的 actions。
- `DeclareLaunchArgument`：声明可以在运行时（命令行）覆盖的参数。
- `LaunchConfiguration`：在 launch 中引用运行时参数的占位符，节点参数处使用它来接收 launch 值。
- `PathJoinSubstitution`：用于安全拼接路径片段（如包 share + 子路径）。
- `FindPackageShare`：查找已安装或工作区内包的 share 目录。
- `Node`：定义并启动一个 ROS 2 节点（指定 package、executable、参数、输出等）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------
    # 默认配置路径：包内的 config/task_plan.yaml
    # 使用 PathJoinSubstitution + FindPackageShare 可以在不同开发/安装场景下正确解析路径
    # ------------------------------------------------------------------
    default_config = PathJoinSubstitution(
        [FindPackageShare("ros2_learning_task_runner"), "config", "task_plan.yaml"]
    )

    # ------------------------------------------------------------------
    # 声明可由命令行覆盖的 launch 参数
    # - task_config: 指向任务 YAML 的路径，节点会读取该 YAML 获得 pickup/dropoff 等位姿
    # - use_sim_time: 是否使用仿真时钟（/clock），在仿真中通常为 true
    # - map_frame/base_frame: TF 坐标系名称
    # - publish_initial_pose/initial_*: 是否发布 /initialpose 及其位姿
    # 注意：DeclareLaunchArgument 的 default_value 可以是 Substitution 对象
    # ------------------------------------------------------------------
    task_config_arg = DeclareLaunchArgument(
        "task_config",
        default_value=default_config,
        description="YAML file with pickup/dropoff poses",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="TF map frame name",
    )

    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="TF base frame name",
    )

    publish_initial_pose_arg = DeclareLaunchArgument(
        "publish_initial_pose",
        default_value="true",
        description="Publish /initialpose if true",
    )

    initial_x_arg = DeclareLaunchArgument(
        "initial_x",
        default_value="0.0",
        description="Initial pose x in map frame",
    )

    initial_y_arg = DeclareLaunchArgument(
        "initial_y",
        default_value="0.0",
        description="Initial pose y in map frame",
    )

    initial_yaw_arg = DeclareLaunchArgument(
        "initial_yaw",
        default_value="0.0",
        description="Initial pose yaw in radians",
    )

    # ------------------------------------------------------------------
    # 启动 task_runner 节点（任务编排）
    # - 将 launch 中的参数以 LaunchConfiguration 注入到节点参数中
    # - 节点内部需通过参数声明/读取接口获取这些值（例如 rclpy/rclcpp 的 declare_parameter/get_parameter）
    # ------------------------------------------------------------------
    task_runner = Node(
        package="ros2_learning_task_runner",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                "task_config": LaunchConfiguration("task_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_initial_pose": LaunchConfiguration("publish_initial_pose"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
            }
        ],
    )

    # ------------------------------------------------------------------
    # 启动 manipulation_stub 节点（抓取/放置占位服务）
    # - 在仿真中用于替代真实机械臂的动作，方便 task_runner 测试任务流程
    # ------------------------------------------------------------------
    manipulation_stub = Node(
        package="ros2_learning_manipulation_stub",
        executable="manipulation_stub",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # 返回 LaunchDescription：包含参数声明（需要先声明）和要启动的节点
    return LaunchDescription(
        [
            task_config_arg,
            use_sim_time_arg,
            map_frame_arg,
            base_frame_arg,
            publish_initial_pose_arg,
            initial_x_arg,
            initial_y_arg,
            initial_yaw_arg,
            manipulation_stub,
            task_runner,
        ]
    )
