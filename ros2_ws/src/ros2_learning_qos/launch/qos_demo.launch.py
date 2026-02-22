from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_learning_qos',
            executable='qos_producer',
            name='mars_rover',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='ros2_learning_qos',
            executable='qos_consumer',
            name='mission_control',
            output='screen',
            emulate_tty=True
        )
    ])
