from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    Launch a container and load Talker and Listener components into it.
    This demonstrates Intra-Process Communication (Zero Copy) potential.
    """

    # 定义一个容器（Component Container）
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 加载 Talker 组件
            ComposableNode(
                package='ros2_learning_cpp',
                plugin='ros2_learning_cpp::TalkerNode',
                name='talker',
                extra_arguments=[{'use_intra_process_comms': True}],  # 开启进程内通信优化
                parameters=[{'message_text': 'Hello from Component'}]
            ),
            # 加载 Listener 组件
            ComposableNode(
                package='ros2_learning_cpp',
                plugin='ros2_learning_cpp::ListenerNode',
                name='listener',
                extra_arguments=[{'use_intra_process_comms': True}]   # 开启进程内通信优化
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
