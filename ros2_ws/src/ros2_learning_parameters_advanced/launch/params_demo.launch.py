"""
参数进阶演示 Launch 文件

启动方式：
  ros2 launch ros2_learning_parameters_advanced params_demo.launch.py
  ros2 launch ros2_learning_parameters_advanced params_demo.launch.py node_label:=my_robot

运行后可动态修改参数：
  ros2 param set /param_demo_node node_label  hello_world
  ros2 param set /param_demo_node scale_factor 0.0   # 会被回调拒绝
  ros2 param set /param_demo_node verbose true

参数优先级（低 → 高）：
  代码默认值 < YAML 文件 < 命令行 node_label 参数（仅当显式传入时）
  注意：CLI 覆盖仅对 node_label 有效；其余参数（scale_factor 等）只能通过 YAML 配置。

实现说明：
  launch_ros 的 Node.parameters 列表在声明时就已固定，无法在运行时按条件删除某项。
  若用 {"node_label": LaunchConfiguration(...)} 且默认值为哨兵字符串，哨兵本身会被写进节点。
  解决办法：用 OpaqueFunction 在上下文求值阶段动态决定是否追加覆盖字典。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 哨兵值：空字符串，表示"用户未传入 node_label"。
# 选用空字符串而非任意占位符，是因为节点业务层本身拒绝空 label，
# 不存在用户真实传入空字符串的合法场景，故不会产生误判。
_UNSET = ""


def _make_param_demo_node(context, *args, **kwargs):
    """OpaqueFunction 回调：在 launch 上下文求值阶段动态构造 param_demo_node。

    只有用户显式传入 node_label 时才将其追加到 parameters 列表，
    否则仅加载 YAML 文件，让 YAML 中的值生效。
    """
    pkg_share = get_package_share_directory("ros2_learning_parameters_advanced")
    default_yaml = os.path.join(pkg_share, "config", "params.yaml")

    # 在上下文中对 LaunchConfiguration 求值，得到实际字符串
    node_label = LaunchConfiguration("node_label").perform(context)

    params = [default_yaml]  # 始终加载 YAML 基础配置
    if node_label != _UNSET:
        # 用户显式传入了 node_label：追加覆盖字典，其优先级高于 YAML
        params.append({"node_label": node_label})

    return [Node(
        package="ros2_learning_parameters_advanced",
        executable="param_demo",
        name="param_demo_node",
        output="screen",
        parameters=params,
    )]


def generate_launch_description():
    node_label_arg = DeclareLaunchArgument(
        "node_label",
        default_value=_UNSET,
        description="覆盖 param_demo_node 的 node_label（不传则使用 YAML 默认值）",
    )

    param_event_listener_node = Node(
        package="ros2_learning_parameters_advanced",
        executable="param_event_listener",
        name="param_event_listener_node",
        output="screen",
    )

    return LaunchDescription([
        node_label_arg,
        OpaqueFunction(function=_make_param_demo_node),
        param_event_listener_node,
    ])
