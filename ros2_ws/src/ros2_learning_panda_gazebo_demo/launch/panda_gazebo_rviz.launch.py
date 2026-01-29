#!/usr/bin/env python3
"""
Panda + Gazebo Sim + RViz2 演示案例 (新手友好).
- Gazebo Sim: 包含简单的世界环境和固定相机
- Panda 模型: 关节位置控制器 (Joint Position Controllers)
- RViz2: 显示机器人模型状态 + 相机实时图像
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取本包的共享目录路径，用于定位模型和其他配置文件
    pkg_share = get_package_share_directory('ros2_learning_panda_gazebo_demo')
    # 获取 ros_gz_sim 包的共享目录路径，因为我们需要使用其中的 launch 文件来启动 Gazebo
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # 定义关键文件的绝对路径
    world_path = os.path.join(pkg_share, 'worlds', 'panda_demo.world') # Gazebo 世界文件
    model_path = os.path.join(pkg_share, 'models', 'panda', 'model.sdf') # Panda 机器人的 SDF 模型
    rviz_path = os.path.join(pkg_share, 'rviz', 'panda_demo.rviz')       # RViz 配置文件
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')     # ROS-Gazebo 桥接配置文件
    urdf_path = os.path.join(pkg_share, 'urdf', 'panda.urdf')           # 机器人的 URDF 描述文件 (给 RViz 用)

    # 读取 URDF 文件内容到字符串中。
    # RobotStatePublisher 需要这个字符串来发布 TF 变换。
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # 设置 Gazebo 的资源路径环境变量。
    # GZ_SIM_RESOURCE_PATH 告诉 Gazebo 在哪里寻找模型文件 (mesh, materials 等)。
    # 我们将本包的 models 目录添加到这个路径中。
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([
            os.path.join(pkg_share, 'models'),
            '/opt/ros/jazzy/share' # 确保 Gazebo 也能找到系统默认的资源
        ])
    )

    # 包含 Gazebo 仿真器的启动文件 (Server 部分)
    # gz_sim.launch.py 是标准入口，我们在参数中传入启动命令。
    # -r: 启动即运行 (Run)
    # -s: 仅运行服务器 (Server)，不带 GUI
    # -v2: 日志等级详细度
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    # 包含 Gazebo 仿真器的启动文件 (GUI 部分)
    # 将 GUI 和 Server 分开启动是常用做法，甚至可以在不同机器上运行。
    # -g: 仅运行 GUI
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # 在 Gazebo 仿真世界中生成 Panda 机器人
    # 使用 ros_gz_sim 包提供的 create 可执行文件。
    # -name: 在仿真中的模型名称
    # -file: 模型文件路径 (SDF)
        # -x, -y, -z: 初始生成位置
    spawn_panda = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'panda', '-file', model_path, '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen',
    )

    # 启动 ROS-Gazebo 桥接节点
    # 这个节点负责在 ROS 2 topic 和 Gazebo transport topic 之间转发消息。
    # 具体的转发规则定义在 bridge.yaml 文件中 (例如 joint states, cmd_pos)。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    # 启动图像桥接节点
    # 专门用于处理图像数据的高效传输 (从 Gazebo 到 ROS)。
    # /demo_camera/image 是 Gazebo 中的 topic 名称 (在 sdf 中定义)。
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/demo_camera/image'],
        output='screen',
    )

    # 启动 Robot State Publisher
    # 它的作用是读取 URDF 并发布机器人的静态 TF 变换，以及根据 joint states 发布动态 TF。
    # 这样 RViz 才能正确显示机器人的姿态。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # 发布一个静态 TF 变换
    # 连接 map/world 坐标系到机器人的基座 (panda_link0)。
    # 虽然这里全是 0，但在多机器人或复杂场景中，这是将机器人固定在世界特定位置的关键。
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0'],
        output='screen',
    )

    # 启动 RViz2 可视化工具
    # -d: 加载指定的配置文件 (*.rviz)。
    # use_sim_time: True 非常重要，表示使用仿真时间而不是系统时间，保证与 Gazebo 同步。
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen',
    )

    # 启动我们自己写的控制器节点 (PandaJointCommander)
    # 它负责发布正弦波控制命令给 Gazebo (通过 bridge)。
    commander = Node(
        package='ros2_learning_panda_gazebo_demo',
        executable='panda_joint_commander',
        output='screen',
    )

    # 延迟启动控制器
    # 这是一个小技巧：等待 3 秒再启动控制器，给 Gazebo 和 Bridge 一点时间初始化。
    # 防止一开始发出的命令因为系统没准备好而丢失。
    delayed_commander = TimerAction(period=3.0, actions=[commander])

    # 打印启动日志
    log_start = LogInfo(
        msg='\n' + '=' * 60 +
            '\n  Panda Gazebo + RViz2 演示程序正在启动...' +
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
