"""
lifecycle_manual.launch.py — 手动 CLI 控制模式

启动两个 LifecycleNode（sensor_node、processor_node），不启动管理器。
节点初始状态为 Unconfigured，需手动通过 ros2 lifecycle CLI 驱动状态转换。

── 启动 ──────────────────────────────────────────────────────────────
  ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py

  可选参数（覆盖 YAML 默认值）：
  ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py \\
    sensor_id:=imu_rear \\
    publish_rate_hz:=5 \\
    configure_behavior:=error     # 测试错误路径

── 手动控制（另开终端）──────────────────────────────────────────────
  # 查看节点可用生命周期转换
  ros2 lifecycle list /sensor_node
  ros2 lifecycle list /processor_node

  # 驱动状态机
  ros2 lifecycle set /sensor_node configure
  ros2 lifecycle set /processor_node configure

  ros2 lifecycle set /sensor_node activate
  ros2 lifecycle set /processor_node activate

  # 观察数据流
  ros2 topic echo /sensor_data
  ros2 topic echo /processed_data

  # 暂停
  ros2 lifecycle set /sensor_node deactivate
  ros2 lifecycle set /processor_node deactivate

  # 重置
  ros2 lifecycle set /sensor_node cleanup
  ros2 lifecycle set /processor_node cleanup

  # 关闭
  ros2 lifecycle set /sensor_node shutdown
  ros2 lifecycle set /processor_node shutdown

── 错误路径测试 ──────────────────────────────────────────────────────
  # 启动时注入 error
  ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py configure_behavior:=error
  # 然后：
  ros2 lifecycle set /processor_node configure
  # 观察日志：on_configure→ERROR → on_error 触发 → 节点回到 Unconfigured
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

_PKG = "ros2_learning_lifecycle"


def generate_launch_description():
    pkg_share = get_package_share_directory(_PKG)
    default_yaml = os.path.join(pkg_share, "config", "lifecycle_demo.yaml")

    # 手动模式下，保留常用可调参数为 launch 参数，便于命令行快速覆盖。
    sensor_id_arg = DeclareLaunchArgument(
        "sensor_id", default_value="lidar_front",
        description="SensorNode 的 sensor_id 参数")
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz", default_value="3",
        description="SensorNode 发布频率（Hz）")
    configure_behavior_arg = DeclareLaunchArgument(
        "configure_behavior", default_value="success",
        description="ProcessorNode configure 行为：success / failure / error")

    # sensor_node（被管理生命周期节点）：
    # - 负责模拟传感器数据发布（/sensor_data）。
    # - 启动后默认处于 Unconfigured，需手动 configure + activate 才会开始工作。
    # - sensor_id / publish_rate_hz 支持 launch 参数覆盖，方便演示不同输入条件。
    sensor_node = LifecycleNode(
        package=_PKG,
        executable="sensor_node",
        name="sensor_node",
        namespace="",
        output="screen",
        parameters=[
            default_yaml,
            {
                "sensor_id":        LaunchConfiguration("sensor_id"),
                "publish_rate_hz":  LaunchConfiguration("publish_rate_hz"),
            },
        ],
    )

    # processor_node（被管理生命周期节点）：
    # - 订阅 /sensor_data 并输出 /processed_data。
    # - 同样默认 Unconfigured，需手动推进生命周期状态机。
    # - configure_behavior 用于演示 success/failure/error 三类配置路径。
    processor_node = LifecycleNode(
        package=_PKG,
        executable="processor_node",
        name="processor_node",
        namespace="",
        output="screen",
        parameters=[
            default_yaml,
            {"configure_behavior": LaunchConfiguration("configure_behavior")},
        ],
    )

    # 本 launch 不启动 lifecycle_manager_node，目的是练习 CLI 手动驱动生命周期。
    # 先注册 launch 参数，再拉起节点进程；真正状态切换顺序由你执行的 CLI 决定。
    return LaunchDescription([
        sensor_id_arg,
        publish_rate_arg,
        configure_behavior_arg,
        sensor_node,
        processor_node,
    ])
