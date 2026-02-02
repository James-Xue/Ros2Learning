#!/usr/bin/env python3
"""
YOLO 检测节点 Launch 文件

使用方法:
    ros2 launch ros2_learning_yolo_detector yolo_detector.launch.py

可选参数:
    model_name:=yolov8n.pt          # 模型名称 (n/s/m/l/x)
    confidence_threshold:=0.5       # 置信度阈值
    image_topic:=/camera/image_raw  # 图像话题
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明 launch 参数
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='yolov8n.pt',
        description='YOLO 模型名称 (yolov8n/s/m/l/x.pt)'
    )

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='检测置信度阈值'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='输入图像话题'
    )

    # 创建 YOLO 检测节点
    yolo_node = Node(
        package='ros2_learning_yolo_detector',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'image_topic': LaunchConfiguration('image_topic'),
        }],
        output='screen',
    )

    return LaunchDescription([
        model_name_arg,
        confidence_arg,
        image_topic_arg,
        yolo_node,
    ])
