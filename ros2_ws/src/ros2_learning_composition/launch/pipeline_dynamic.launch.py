"""
动态加载演示：启动一个空的 ComposableNodeContainer，
用户可在运行时通过 ros2 component CLI 手动加载组件。

运行方式：
  # 终端 A：启动空容器
  ros2 launch ros2_learning_composition pipeline_dynamic.launch.py

  # 终端 B：查看当前容器
  ros2 component list

  # 手动加载组件（按流水线顺序）
  ros2 component load /pipeline_container ros2_learning_composition \
      ros2_learning_composition::ImageProducer \
      --node-name image_producer \
      -e use_intra_process_comms:=true

  ros2 component load /pipeline_container ros2_learning_composition \
      ros2_learning_composition::ImageProcessor \
      --node-name image_processor \
      -e use_intra_process_comms:=true

  ros2 component load /pipeline_container ros2_learning_composition \
      ros2_learning_composition::ImageSaver \
      --node-name image_saver \
      -e use_intra_process_comms:=true

  # 查看已加载组件
  ros2 component list

  # 卸载组件（通过组件 ID）
  ros2 component unload /pipeline_container <component_id>
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """生成空容器，等待动态加载组件。"""
    container = ComposableNodeContainer(
        name="pipeline_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([container])
