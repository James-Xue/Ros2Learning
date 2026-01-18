#!/usr/bin/env python3
"""
四元数和 TF 演示 Launch 文件

启动所有演示节点和 RViz 可视化
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Launch 描述"""
    
    # RViz 配置文件路径
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ros2_learning_tf_quaternion_demo'),
        'rviz',
        'demo.rviz'
    ])
    
    return LaunchDescription([
        # 欢迎信息
        LogInfo(msg='\n' + '='*60 + \
                '\n  🎓 ROS 2 四元数和 TF 变换演示' + \
                '\n' + '='*60),
        
        LogInfo(msg='[1/5] 启动静态 TF 广播器（世界坐标系）...'),
        
        # 静态 TF: world 坐标系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
            output='screen'
        ),
        
        LogInfo(msg='[2/5] 启动四元数可视化演示节点...'),
        
        # 四元数演示节点
        Node(
            package='ros2_learning_tf_quaternion_demo',
            executable='quaternion_demo',
            name='quaternion_demo',
            output='screen'
        ),
        
        LogInfo(msg='[3/5] 启动 TF 广播器演示节点...'),
        
        # TF 广播器演示
        Node(
            package='ros2_learning_tf_quaternion_demo',
            executable='tf_broadcaster_demo',
            name='tf_broadcaster_demo',
            output='screen'
        ),
        
        LogInfo(msg='[4/5] 启动 TF 监听器演示节点...'),
        
        # TF 监听器演示
        Node(
            package='ros2_learning_tf_quaternion_demo',
            executable='tf_listener_demo',
            name='tf_listener_demo',
            output='screen'
        ),
        
        LogInfo(msg='[5/5] 启动 RViz 可视化...'),
        
        # RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        LogInfo(msg='\n✅ 所有节点已启动！\n' + \
                '📊 在 RViz 中观察坐标系和四元数可视化\n' + \
                '📝 查看终端输出了解四元数和 TF 的概念\n')
    ])
