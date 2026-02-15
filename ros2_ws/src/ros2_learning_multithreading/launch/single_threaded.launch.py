from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    启动单线程执行器演示。
    在此模式下，耗时计算回调会阻塞心跳定时器。
    """
    return LaunchDescription([
        Node(
            package='ros2_learning_multithreading',
            executable='executor_demo',
            name='single_threaded_demo',
            parameters=[{'executor_type': 'single'}],
            output='screen'
        )
    ])
