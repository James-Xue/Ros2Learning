"""!
@file count_action_demo.launch.py
@brief CountActionServer + CountActionClient 演示启动文件。
@details
同时启动 count_action_server 和 count_action_client。
支持通过 launch 参数配置目标、周期和取消超时。

启动（默认：target=5, period=0.3s, 不取消）：
  ros2 launch ros2_learning_actions count_action_demo.launch.py

演示取消路径（2 秒后自动取消）：
  ros2 launch ros2_learning_actions count_action_demo.launch.py \\
      target:=20 period_sec:=0.5 cancel_after_sec:=2.0

观察（另开终端）：
  ros2 action list
  ros2 action info /count_task -t
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PKG = "ros2_learning_actions"


def generate_launch_description():
    """!
    @brief 生成 count_action_demo LaunchDescription。
    @return launch.LaunchDescription 启动描述对象。
    """
    # ── Launch 参数声明 ──────────────────────────────────────────────────
    target_arg = DeclareLaunchArgument(
        "target", default_value="5",
        description="计数目标（必须 > 0）")

    period_sec_arg = DeclareLaunchArgument(
        "period_sec", default_value="0.3",
        description="每次计数的等待时间（秒，必须 > 0）")

    cancel_after_sec_arg = DeclareLaunchArgument(
        "cancel_after_sec", default_value="-1.0",
        description="客户端发送取消前的延迟（秒），负值表示不取消")

    # ── count_action_server ────────────────────────────────────────────
    # 等待 Client 发送 goal，校验参数，执行计数循环，发布 Feedback，返回 Result。
    server_node = Node(
        package=_PKG,
        executable="count_action_server",
        name="count_action_server",
        namespace="",
        output="screen",
    )

    # ── count_action_client ────────────────────────────────────────────
    # 从参数读取 goal 配置，等待 server 就绪后发送 goal，打印 Feedback 和 Result。
    client_node = Node(
        package=_PKG,
        executable="count_action_client",
        name="count_action_client",
        namespace="",
        output="screen",
        parameters=[{
            "target": LaunchConfiguration("target"),
            "period_sec": LaunchConfiguration("period_sec"),
            "cancel_after_sec": LaunchConfiguration("cancel_after_sec"),
        }],
    )

    return LaunchDescription([
        target_arg,
        period_sec_arg,
        cancel_after_sec_arg,
        server_node,
        client_node,
    ])
