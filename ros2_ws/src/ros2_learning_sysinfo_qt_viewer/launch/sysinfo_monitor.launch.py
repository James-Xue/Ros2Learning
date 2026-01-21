#!/usr/bin/env python3
"""
系统信息监控 Launch 文件

同时启动系统信息发布器和 Qt 查看器
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""
    
    return LaunchDescription([
        # 欢迎信息
        LogInfo(msg='\n' + '='*60 + \
                '\n  📊 ROS 2 系统信息监控' + \
                '\n' + '='*60),
        
        LogInfo(msg='[1/2] 启动系统信息发布器...'),
        
        # 系统信息发布器
        Node(
            package='ros2_learning_sysinfo_publisher',
            executable='sysinfo_publisher',
            name='sysinfo_publisher',
            output='screen',
            parameters=[{
                'topic': '/ros2_learning/sysinfo',
                'publish_rate_hz': 1.0
            }]
        ),
        
        LogInfo(msg='[2/2] 启动 Qt 监控界面...'),
        
        # Qt 查看器
        Node(
            package='ros2_learning_sysinfo_qt_viewer',
            executable='sysinfo_qt_viewer',
            name='sysinfo_qt_viewer',
            output='screen',
            parameters=[{
                'topic': '/ros2_learning/sysinfo'
            }]
        ),
        
        LogInfo(msg='\n✅ 所有节点已启动！\n' + \
                '📊 Qt 窗口将显示系统信息\n' + \
                '💡 按 Ctrl+C 关闭所有节点\n')
    ])
