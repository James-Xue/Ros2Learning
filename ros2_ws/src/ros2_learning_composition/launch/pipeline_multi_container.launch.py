"""
双容器对比演示：将 Producer 和 Processor+Saver 分到两个不同的容器，
观察跨容器通信走 DDS 序列化（零拷贝失效）。
启动后自动 configure + activate ImageSaver。

运行方式：
  ros2 launch ros2_learning_composition pipeline_multi_container.launch.py

对比要点：
  - container_a（Producer）和 container_b（Processor+Saver）之间通过 DDS 通信
  - 即使启用了 use_intra_process_comms，跨容器的消息仍需序列化/反序列化
  - Processor→Saver 在同一容器内，仍可零拷贝
  - 对比 pipeline_composed.launch.py 的延迟差异
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    """生成双容器流水线启动描述。"""

    # 容器 A：仅包含 ImageProducer
    container_a = ComposableNodeContainer(
        name="container_producer",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2_learning_composition",
                plugin="ros2_learning_composition::ImageProducer",
                name="image_producer",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[{
                    "image_width": 64,
                    "image_height": 64,
                    "publish_rate_hz": 10.0,
                }],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    # 容器 B：包含 ImageProcessor + ImageSaver
    container_b = ComposableNodeContainer(
        name="container_consumer",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2_learning_composition",
                plugin="ros2_learning_composition::ImageProcessor",
                name="image_processor",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="ros2_learning_composition",
                plugin="ros2_learning_composition::ImageSaver",
                name="image_saver",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    # 延迟后自动 configure + activate ImageSaver
    configure_saver = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/image_saver", "configure"],
                output="screen",
            ),
        ],
    )

    activate_saver = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/image_saver", "activate"],
                output="screen",
            ),
        ],
    )

    return launch.LaunchDescription([
        container_a, container_b, configure_saver, activate_saver
    ])
