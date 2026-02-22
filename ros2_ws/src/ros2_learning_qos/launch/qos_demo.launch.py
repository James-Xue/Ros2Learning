from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    使用 ROS 2 Composition 将两个节点加载到同一个进程中。
    好处：
    1. 同进程内通信可实现零拷贝 (Zero-copy / Intra-process)
    2. 减少进程数量，节约系统资源
    3. 节点仍然可以单独运行 (ros2 run ros2_learning_qos qos_producer)
    """
    container = ComposableNodeContainer(
        name='qos_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_learning_qos',
                plugin='ros2_learning_qos::RoverNode',
                name='mars_rover',
            ),
            ComposableNode(
                package='ros2_learning_qos',
                plugin='ros2_learning_qos::MissionControlNode',
                name='mission_control',
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([container])
