from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    启动行为树演示节点。
    
    参数:
    - tree_file: 要加载的行为树 XML 文件名 (相对于 behavior_trees 目录)
      默认值: main_tree_composition.xml (这是我们刚拆分好的主树)
    """
    
    # 声明启动参数
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='main_tree_composition.xml',
        description='The behavior tree XML file to load'
    )

    tree_file = LaunchConfiguration('tree_file')

    bt_node = Node(
        package='ros2_learning_behavior_tree',
        executable='bt_main',
        name='bt_demo_node',
        output='screen',
        parameters=[{'tree_file': tree_file}]
    )

    return LaunchDescription([
        tree_file_arg,
        bt_node
    ])
