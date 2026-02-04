#!/usr/bin/env python3
"""
demo.launch.py
极简 ros2_control 示例的启动文件

启动流程:
=========
1. 加载 URDF 并发布到 /robot_description
2. 启动 Controller Manager（加载 Hardware Interface）
3. 启动 Joint State Broadcaster（发布 /joint_states）
4. 启动 Forward Position Controller（接收位置命令）
5. 启动 Robot State Publisher（发布 TF）

使用方法:
=========
ros2 launch ros2_learning_control_demo demo.launch.py

发送命令:
=========
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('ros2_learning_control_demo')

    # URDF 文件路径
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'simple_joint.urdf.xacro'])

    # 控制器配置文件路径
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # 使用 xacro 处理 URDF
    robot_description = Command(['xacro ', urdf_file])

    # ============================================================
    # 节点定义
    # ============================================================

    # Robot State Publisher: 发布机器人状态到 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

    # Controller Manager: 核心节点，管理所有控制器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controller_config,
        ],
        output='screen',
    )

    # 启动 Joint State Broadcaster
    # 使用 spawner 工具来启动控制器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # 启动 Forward Position Controller
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ])
