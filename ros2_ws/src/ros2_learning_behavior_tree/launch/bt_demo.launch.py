#!/usr/bin/env python3

import os

# 用于查找包安装路径（比如 XML 文件在哪里）
from ament_index_python.packages import get_package_share_directory
# LaunchDescription 是整个启动配置的容器
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# Node 是用来启动 ROS 2 节点的动作
from launch_ros.actions import Node

# 这里的函数名是固定的！ROS 2 找这一行作为入口点
def generate_launch_description():
    # 获取此包被安装后的 share 目录路径
    # 例如：/root/Ros2Learning/ros2_ws/install/ros2_learning_behavior_tree/share/ros2_learning_behavior_tree
    pkg_share = get_package_share_directory('ros2_learning_behavior_tree')
    
    # 也可以提供参数来动态选择 XML 文件
    # xml_file = LaunchConfiguration('xml_file')
    
    # 定义我们要启动的节点
    bt_node = Node(
        # 包名：去哪个包找可执行文件
        package='ros2_learning_behavior_tree',
        
        # 可执行文件名：CMakeLists.txt 中 add_executable(bt_main ...) 定义的名字
        executable='bt_main',
        
        # 输出模式：'screen' 表示把日志打印到终端（不设置的话你看不到 printf/cout）
        output='screen',
        
        # 模拟 TTY：这对 BehaviorTree.CPP 很重要！
        # 它可以让日志保留颜色（比如红色的 FAILURE，绿色的 SUCCESS）
        # 如果没有这一行，日志可能会是一堆没有颜色的纯文本，甚至被缓冲导致显示延迟
        emulate_tty=True
    )

    # 返回描述对象，告诉 ROS 2 启动器“请帮我执行列表里的所有动作”
    return LaunchDescription([
        bt_node
    ])
