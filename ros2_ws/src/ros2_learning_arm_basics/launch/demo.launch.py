#!/usr/bin/env python3
"""
Panda机械臂演示Launch文件

整合了MoveIt 2仿真环境和arm_position_controller控制节点
一键启动完整的机械臂控制演示环境

使用方法:
    ros2 launch ros2_learning_arm_basics demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
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
    # 4. 添加日志信息
    # ═══════════════════════════════════════════════════════════
    log_start = LogInfo(
        msg='\n' + '='*60 + \
            '\n  🤖 Panda机械臂控制演示 - 启动中...' + \
            '\n' + '='*60
    )
    
    log_moveit_starting = LogInfo(
        msg='[步骤 1/3] 🚀 正在启动 MoveIt 2 仿真环境...\n' + \
            '           - RViz2 可视化界面\n' + \
            '           - MoveGroup 运动规划服务\n' + \
            '           - ros2_control 仿真器'
    )
    
    log_waiting = LogInfo(
        msg='[步骤 2/3] ⏳ 等待 5 秒，确保 MoveGroup 服务完全就绪...'
    )
    
    log_controller_starting = LogInfo(
        msg='[步骤 3/3] 🎯 启动机械臂位置控制节点...\n' + \
            '           节点将自动执行演示动作序列！'
    )
    
    # 将日志添加到延迟启动序列中
    delayed_arm_controller = TimerAction(
        period=5.0,  # 延迟5秒
        actions=[log_controller_starting, arm_controller]
    )
    
    # ═══════════════════════════════════════════════════════════
    # 5. 组装Launch描述
    # ═══════════════════════════════════════════════════════════
    return LaunchDescription([
        log_start,              # 欢迎信息
        log_moveit_starting,    # 提示启动仿真环境
        moveit_demo,            # 启动仿真环境
        log_waiting,            # 提示等待中
        delayed_arm_controller  # 延迟启动控制节点（带日志）
    ])
