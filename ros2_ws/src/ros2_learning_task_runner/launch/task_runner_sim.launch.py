from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("ros2_learning_task_runner"), "config", "task_plan.yaml"]
    )

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

    task_runner = Node(
        package="ros2_learning_task_runner",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                "task_config": LaunchConfiguration("task_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    manipulation_stub = Node(
        package="ros2_learning_manipulation_stub",
        executable="manipulation_stub",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([task_config_arg, use_sim_time_arg, manipulation_stub, task_runner])
