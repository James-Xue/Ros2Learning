"""
单容器组合启动：三个组件加载到同一个 ComposableNodeContainer，
启用 use_intra_process_comms=True 实现 intra-process 零拷贝通信。
启动后自动 configure + activate ImageSaver 以形成完整流水线。

运行方式：
  ros2 launch ros2_learning_composition pipeline_composed.launch.py

观察要点：
  - ImageProducer 和 ImageProcessor 的 DEBUG 日志会打印消息 data 指针地址
  - 在同一容器内，相邻节点间的指针地址应相同（零拷贝生效）
  - 启用 DEBUG 日志：--ros-args --log-level debug
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    """生成单容器零拷贝流水线启动描述。"""
    container = ComposableNodeContainer(
        name="pipeline_container",
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

    # 延迟 2 秒后自动 configure + activate ImageSaver
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

    return launch.LaunchDescription([container, configure_saver, activate_saver])
