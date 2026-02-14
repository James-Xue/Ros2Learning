from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    message_text_arg = DeclareLaunchArgument(
        'message_text',
        default_value='Hello ROS 2 from Launch',
        description='The text to publish'
    )

    # 获取参数值
    message_text = LaunchConfiguration('message_text')

    # 定义节点
    talker_node = Node(
        package='ros2_learning_cpp',
        executable='talker',
        name='my_talker',
        output='screen',
        parameters=[
            {'message_text': message_text}
        ]
    )

    return LaunchDescription([
        message_text_arg,
        talker_node
    ])
