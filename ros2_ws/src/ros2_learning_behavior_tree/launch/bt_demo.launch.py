from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    ROS 2 Launch 脚本：用于便捷启动带参数的行为树演示节点。
    
    知识点：
    1. DeclareLaunchArgument: 定义 launch 文件的“接口”，允许用户在终端通过 变量:=值 的方式修改。
    2. LaunchConfiguration: 用于在脚本中引用上述用户传入或默认的参数值。
    3. parameters: 将 Launch 层级的参数传递给节点的私有参数空间（用于 tree_file 的动态切换）。
    """
    
    # 1. 声明启动参数 tree_file
    # 用户可以在启动时指定：ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=mock_fallback_demo.xml
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='main_tree_composition.xml',
        description='要加载的行为树 XML 文件名 (存放在 behavior_trees 目录下)'
    )

    # 2. 获取参数值的引用
    tree_file = LaunchConfiguration('tree_file')

    # 3. 定义要启动的节点动作
    bt_node = Node(
        package='ros2_learning_behavior_tree',
        executable='bt_main',
        name='bt_demo_node',
        output='screen', # 将节点的标准输出打印到控制台
        # 核心：将 launch 参数映射到节点在 bt_main.cpp 中声明的 'tree_file' 参数
        parameters=[{'tree_file': tree_file}]
    )

    # 4. 组装并返回描述对象
    return LaunchDescription([
        tree_file_arg,
        bt_node
    ])
