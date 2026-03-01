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

    # sensor_node（被管理生命周期节点）：
    # - 模拟传感器源，周期发布 /sensor_data。
    # - 仅在被 manager activate 后才会真正开始对外发布数据。
    # - 处于 inactive/unconfigured 时，节点存在但不会输出业务数据。
    sensor_node = LifecycleNode(
        package=_PKG,
        executable="sensor_node",
        name="sensor_node",
        namespace="",
        output="screen",
        parameters=[default_yaml],
    )

    # processor_node（被管理生命周期节点）：
    # - 订阅 /sensor_data，做简单处理后发布 /processed_data。
    # - 与 sensor_node 一样，只有在 activate 后才进入“工作态”。
    # - 通过生命周期状态切换，可观察配置阶段与运行阶段的差异。
    processor_node = LifecycleNode(
        package=_PKG,
        executable="processor_node",
        name="processor_node",
        namespace="",
        output="screen",
        parameters=[default_yaml],
    )

    # lifecycle_manager_node（普通 Node，管理者）：
    # - 不是 LifecycleNode，本身不参与生命周期状态机。
    # - 通过 /<node>/get_state 和 /<node>/change_state 服务，
    #   按顺序驱动 sensor/processor 完成 configure -> activate。
    # - 负责整体启动编排，便于演示“集中式生命周期管理”模式。
    lifecycle_manager = Node(
        package=_PKG,
        executable="lifecycle_manager_node",
        name="lifecycle_manager_node",
        output="screen",
        parameters=[default_yaml],
    )

    # 启动顺序只影响进程拉起，不等同于生命周期状态切换顺序；
    # 真正的 configure/activate 顺序由 lifecycle_manager_node 内部逻辑决定。
    return LaunchDescription([
        sensor_node,
        processor_node,
        lifecycle_manager,
    ])
