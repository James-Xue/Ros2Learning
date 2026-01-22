from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_learning_tb3_spawner",
                executable="tb3_spawner",
                name="tb3_spawner",
                output="screen",
            )
        ]
    )
