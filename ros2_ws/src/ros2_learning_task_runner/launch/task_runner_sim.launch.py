"""Task runner + manipulation stub 启动脚本（仿真模式）"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 默认任务配置文件路径：<pkg_share>/config/task_plan.yaml
    default_config = PathJoinSubstitution(
        [FindPackageShare("ros2_learning_task_runner"), "config", "task_plan.yaml"]
    )

    # 允许在命令行覆盖 task_config
    task_config_arg = DeclareLaunchArgument(
        "task_config",
        default_value=default_config,
        description="YAML file with pickup/dropoff poses",
    )
    # 是否使用仿真时钟 /clock
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    # 启动任务编排节点
    task_runner = Node(
        package="ros2_learning_task_runner",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                # 将 launch 参数传入节点参数
                "task_config": LaunchConfiguration("task_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # 启动抓取/放置占位服务节点
    manipulation_stub = Node(
        package="ros2_learning_manipulation_stub",
        executable="manipulation_stub",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # 返回启动描述（参数声明 + 两个节点）
    return LaunchDescription([task_config_arg, use_sim_time_arg, manipulation_stub, task_runner])
