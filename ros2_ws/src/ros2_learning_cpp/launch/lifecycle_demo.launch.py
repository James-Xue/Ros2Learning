from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LifecycleNode
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    1. Launch a ComposableNodeContainer with a standard Listener.
    2. Launch a Lifecycle Talker (default state: Unconfigured).
    
    You need to manually manage the lifecycle of /talker_lifecycle:
    - ros2 lifecycle set /talker_lifecycle configure
    - ros2 lifecycle set /talker_lifecycle activate
    - ros2 lifecycle set /talker_lifecycle deactivate
    - ros2 lifecycle set /talker_lifecycle cleanup
    - ros2 lifecycle set /talker_lifecycle shutdown
    """

    # 1. Listener 组件 (在容器内，随时待命)
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_learning_cpp',
                plugin='ros2_learning_cpp::ListenerNode',
                name='listener',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    # 2. Lifecycle Talker 节点
    # 注意：这里我们用 executable='talker'，这是 rclcpp_components_register_node 自动生成的
    # 它本质上是一个加载 Talker 组件的壳，但我们在这里把它当作独立的 LifecycleNode 启动
    talker_lifecycle_node = LifecycleNode(
        package='ros2_learning_cpp',
        executable='talker',
        name='talker_lifecycle',
        namespace='',
        output='screen',
        parameters=[{'message_text': 'Hello Lifecycle Manual Mode'}],
    )

    return LaunchDescription([
        container,
        talker_lifecycle_node,
    ])
