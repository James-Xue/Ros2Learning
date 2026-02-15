from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    启动多线程执行器演示。
    在此模式下，心跳定时器与耗时计算回调互不干扰（因为它们在不同的 CallbackGroup 中）。
    """
    return LaunchDescription([
        Node(
            package='ros2_learning_multithreading',
            executable='executor_demo',
            name='multi_threaded_demo',
            parameters=[{'executor_type': 'multi'}],
            output='screen'
        )
    ])
