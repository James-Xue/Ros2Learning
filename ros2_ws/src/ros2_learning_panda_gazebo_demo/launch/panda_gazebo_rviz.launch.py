#!/usr/bin/env python3
"""
Panda + Gazebo Sim + RViz2 demo (beginner-friendly).
- Gazebo Sim: world + fixed camera
- Panda model: joint position controllers
- RViz2: robot model + camera image
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_learning_panda_gazebo_demo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'panda_demo.world')
    model_path = os.path.join(pkg_share, 'models', 'panda', 'model.sdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'panda_demo.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'panda.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([
            os.path.join(pkg_share, 'models'),
            '/opt/ros/jazzy/share'
        ])
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_panda = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'panda', '-file', model_path, '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/demo_camera/image'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0'],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen',
    )

    commander = Node(
        package='ros2_learning_panda_gazebo_demo',
        executable='panda_joint_commander',
        output='screen',
    )

    delayed_commander = TimerAction(period=3.0, actions=[commander])

    log_start = LogInfo(
        msg='\n' + '=' * 60 +
            '\n  Panda Gazebo + RViz2 demo starting...' +
            '\n' + '=' * 60
    )

    return LaunchDescription([
        log_start,
        set_gz_resource_path,
        gz_server,
        gz_gui,
        spawn_panda,
        bridge,
        camera_bridge,
        robot_state_publisher,
        static_tf,
        rviz2,
        delayed_commander,
    ])
