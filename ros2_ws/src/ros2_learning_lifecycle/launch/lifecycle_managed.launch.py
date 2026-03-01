"""
lifecycle_managed.launch.py — LifecycleManager 自动管理模式

同时启动 sensor_node、processor_node 和 lifecycle_manager_node。
manager 会在启动约 1 秒后自动顺序执行：
  1. configure sensor_node → configure processor_node
  2. 等待 transition_delay_sec
  3. activate sensor_node → activate processor_node

── 启动 ──────────────────────────────────────────────────────────────
  ros2 launch ros2_learning_lifecycle lifecycle_managed.launch.py

── 观察（另开终端）──────────────────────────────────────────────────
  ros2 topic echo /sensor_data       # 应在 activate 后出现数据
  ros2 topic echo /processed_data    # 应在 activate 后出现平均值
  ros2 lifecycle list /sensor_node   # 确认节点处于 active 状态
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

_PKG = "ros2_learning_lifecycle"


def generate_launch_description():
    pkg_share = get_package_share_directory(_PKG)
    default_yaml = os.path.join(pkg_share, "config", "lifecycle_demo.yaml")

    sensor_node = LifecycleNode(
        package=_PKG,
        executable="sensor_node",
        name="sensor_node",
        namespace="",
        output="screen",
        parameters=[default_yaml],
    )

    processor_node = LifecycleNode(
        package=_PKG,
        executable="processor_node",
        name="processor_node",
        namespace="",
        output="screen",
        parameters=[default_yaml],
    )

    lifecycle_manager = Node(
        package=_PKG,
        executable="lifecycle_manager_node",
        name="lifecycle_manager_node",
        output="screen",
        parameters=[default_yaml],
    )

    return LaunchDescription([
        sensor_node,
        processor_node,
        lifecycle_manager,
    ])
