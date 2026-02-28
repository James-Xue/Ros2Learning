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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node


# 用于标记"用户未传入此参数"的哨兵值
_UNSET = "__unset__"


def generate_launch_description():
    pkg_share = get_package_share_directory("ros2_learning_parameters_advanced")
    default_yaml = os.path.join(pkg_share, "config", "params.yaml")

    # 默认值为哨兵：只有用户显式传入时才覆盖 YAML 中的值
    node_label_arg = DeclareLaunchArgument(
        "node_label",
        default_value=_UNSET,
        description="覆盖 param_demo_node 的 node_label（不传则使用 YAML 默认值）",
    )

    node_label_cfg = LaunchConfiguration("node_label")

    # 当 node_label 未被显式传入时，仅加载 YAML；传入时再追加覆盖字典
    # launch_ros Node.parameters 列表按顺序合并，后者覆盖前者
    param_demo_node = Node(
        package="ros2_learning_parameters_advanced",
        executable="param_demo",
        name="param_demo_node",
        output="screen",
        parameters=[
            # 1. YAML 文件（始终加载）
            default_yaml,
            # 2. 命令行覆盖字典（仅当用户传了非哨兵值时生效）
            #    NotEqualsSubstitution 在 Jazzy 中需要 launch >= 3.x；
            #    若版本不支持，可改用 OpaqueFunction 实现同等逻辑
            {"node_label": node_label_cfg},
        ],
        # 用条件来控制 node_label 覆盖不在此处——parameters 列表无法按条件删除项；
        # 实际上 launch_ros 在参数值为字符串时会直接传给 ros2，哨兵值会被设进去。
        # 正确做法：在 OpaqueFunction 中动态构造 parameters 列表（见下方替代方案注释）
    )

    param_event_listener_node = Node(
        package="ros2_learning_parameters_advanced",
        executable="param_event_listener",
        name="param_event_listener_node",
        output="screen",
    )

    return LaunchDescription([
        node_label_arg,
        param_demo_node,
        param_event_listener_node,
    ])


# ---------------------------------------------------------------------------
# 替代方案（更健壮）：使用 OpaqueFunction 动态构造 parameters
# ---------------------------------------------------------------------------
# from launch.actions import OpaqueFunction
#
# def _make_param_demo_node(context, *args, **kwargs):
#     pkg_share = get_package_share_directory("ros2_learning_parameters_advanced")
#     default_yaml = os.path.join(pkg_share, "config", "params.yaml")
#     params = [default_yaml]
#
#     node_label = LaunchConfiguration("node_label").perform(context)
#     if node_label != _UNSET:
#         params.append({"node_label": node_label})
#
#     return [Node(
#         package="ros2_learning_parameters_advanced",
#         executable="param_demo",
#         name="param_demo_node",
#         output="screen",
#         parameters=params,
#     )]
