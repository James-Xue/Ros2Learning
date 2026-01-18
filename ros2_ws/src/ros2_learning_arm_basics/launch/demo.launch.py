#!/usr/bin/env python3
"""
Panda机械臂演示Launch文件

整合了MoveIt 2仿真环境和arm_position_controller控制节点
一键启动完整的机械臂控制演示环境

使用方法:
    ros2 launch ros2_learning_arm_basics demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """生成Launch描述"""
    
    # ═══════════════════════════════════════════════════════════
    # 1. 引入Panda机械臂的MoveIt 2仿真环境
    # ═══════════════════════════════════════════════════════════
    # 这个launch文件包含：
    # - RViz2可视化
    # - MoveGroup节点（运动规划服务）
    # - ros2_control仿真器
    # - 机器人状态发布器
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_resources_panda_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ])
    )
    
    # ═══════════════════════════════════════════════════════════
    # 2. 定义机械臂位置控制节点
    # ═══════════════════════════════════════════════════════════
    arm_controller = Node(
        package='ros2_learning_arm_basics',
        executable='arm_position_controller',
        name='arm_position_controller',
        output='screen',
        parameters=[
            {'planning_group': 'panda_arm'}  # 规划组名称
        ]
    )
    
    # ═══════════════════════════════════════════════════════════
    # 3. 延迟启动控制节点
    # ═══════════════════════════════════════════════════════════
    # 等待5秒，确保MoveGroup服务完全就绪
    # 这样可以避免控制节点启动时找不到MoveGroup服务
    delayed_arm_controller = TimerAction(
        period=5.0,  # 延迟5秒
        actions=[arm_controller]
    )
    
    # ═══════════════════════════════════════════════════════════
    # 4. 组装Launch描述
    # ═══════════════════════════════════════════════════════════
    return LaunchDescription([
        moveit_demo,           # 先启动仿真环境
        delayed_arm_controller # 延迟启动控制节点
    ])
