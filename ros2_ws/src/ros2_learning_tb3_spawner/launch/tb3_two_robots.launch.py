import os
import tempfile
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _patch_sdf(xml: str, name: str) -> str:
    # Make Gazebo transport topics unique per robot by prefixing with /<name>/...
    # This keeps the multi-robot tutorial simple and avoids topic collisions.
    replacements = {
        "<topic>cmd_vel</topic>": f"<topic>/{name}/cmd_vel</topic>",
        "<odom_topic>odom</odom_topic>": f"<odom_topic>/{name}/odom</odom_topic>",
        "<tf_topic>/tf</tf_topic>": f"<tf_topic>/{name}/tf</tf_topic>",
        "<topic>joint_states</topic>": f"<topic>/{name}/joint_states</topic>",
        "<topic>imu</topic>": f"<topic>/{name}/imu</topic>",
        "<topic>scan</topic>": f"<topic>/{name}/scan</topic>",
        "<topic>camera/image_raw</topic>": f"<topic>/{name}/camera/image_raw</topic>",
        "<camera_info_topic>camera/camera_info</camera_info_topic>": f"<camera_info_topic>/{name}/camera/camera_info</camera_info_topic>",
    }

    for src, dst in replacements.items():
        xml = xml.replace(src, dst)

    return xml


def _write_bridge_yaml(path: str, name: str, include_camera: bool) -> None:
    # ros_gz_bridge parameter_bridge config file
    entries: List[str] = [
        # Do NOT bridge /clock here; turtlebot3_world already bridges it.
        "- ros_topic_name: \"/{name}/joint_states\"\n"
        "  gz_topic_name: \"/{name}/joint_states\"\n"
        "  ros_type_name: \"sensor_msgs/msg/JointState\"\n"
        "  gz_type_name: \"gz.msgs.Model\"\n"
        "  direction: GZ_TO_ROS\n".format(name=name),
        "- ros_topic_name: \"/{name}/odom\"\n"
        "  gz_topic_name: \"/{name}/odom\"\n"
        "  ros_type_name: \"nav_msgs/msg/Odometry\"\n"
        "  gz_type_name: \"gz.msgs.Odometry\"\n"
        "  direction: GZ_TO_ROS\n".format(name=name),
        "- ros_topic_name: \"/{name}/tf\"\n"
        "  gz_topic_name: \"/{name}/tf\"\n"
        "  ros_type_name: \"tf2_msgs/msg/TFMessage\"\n"
        "  gz_type_name: \"gz.msgs.Pose_V\"\n"
        "  direction: GZ_TO_ROS\n".format(name=name),
        "- ros_topic_name: \"/{name}/cmd_vel\"\n"
        "  gz_topic_name: \"/{name}/cmd_vel\"\n"
        "  ros_type_name: \"geometry_msgs/msg/TwistStamped\"\n"
        "  gz_type_name: \"gz.msgs.Twist\"\n"
        "  direction: ROS_TO_GZ\n".format(name=name),
        "- ros_topic_name: \"/{name}/imu\"\n"
        "  gz_topic_name: \"/{name}/imu\"\n"
        "  ros_type_name: \"sensor_msgs/msg/Imu\"\n"
        "  gz_type_name: \"gz.msgs.IMU\"\n"
        "  direction: GZ_TO_ROS\n".format(name=name),
        "- ros_topic_name: \"/{name}/scan\"\n"
        "  gz_topic_name: \"/{name}/scan\"\n"
        "  ros_type_name: \"sensor_msgs/msg/LaserScan\"\n"
        "  gz_type_name: \"gz.msgs.LaserScan\"\n"
        "  direction: GZ_TO_ROS\n".format(name=name),
    ]

    if include_camera:
        entries.append(
            "- ros_topic_name: \"/{name}/camera/camera_info\"\n"
            "  gz_topic_name: \"/{name}/camera/camera_info\"\n"
            "  ros_type_name: \"sensor_msgs/msg/CameraInfo\"\n"
            "  gz_type_name: \"gz.msgs.CameraInfo\"\n"
            "  direction: GZ_TO_ROS\n".format(name=name)
        )

    with open(path, "w", encoding="utf-8") as f:
        f.write("# Auto-generated for multi-robot TB3 spawning\n")
        f.writelines(entries)


def _spawn_second_robot(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context)
    name = LaunchConfiguration("second_name").perform(context)
    x = LaunchConfiguration("second_x").perform(context)
    y = LaunchConfiguration("second_y").perform(context)
    yaw = LaunchConfiguration("second_yaw").perform(context)

    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    model_sdf_path = os.path.join(
        tb3_gazebo_share, "models", f"turtlebot3_{model}", "model.sdf"
    )

    with open(model_sdf_path, "r", encoding="utf-8") as f:
        xml = f.read()

    xml = _patch_sdf(xml, name)

    tmp_dir = tempfile.gettempdir()
    patched_sdf = os.path.join(tmp_dir, f"ros2_learning_tb3_{name}.sdf")
    bridge_yaml = os.path.join(tmp_dir, f"ros2_learning_tb3_{name}_bridge.yaml")

    with open(patched_sdf, "w", encoding="utf-8") as f:
        f.write(xml)

    include_camera = model != "burger"
    _write_bridge_yaml(bridge_yaml, name, include_camera)

    actions = [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                name,
                "-file",
                patched_sdf,
                "-x",
                x,
                "-y",
                y,
                "-z",
                "0.01",
                "-Y",
                yaw,
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["--ros-args", "-p", f"config_file:={bridge_yaml}"],
            output="screen",
        ),
    ]

    if include_camera:
        actions.append(
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                arguments=[f"/{name}/camera/image_raw"],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    model = LaunchConfiguration("model")

    # Start world + first robot (official launch)
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    world_launch = os.path.join(tb3_gazebo_share, "launch", "turtlebot3_world.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="waffle_pi"),
            DeclareLaunchArgument("second_name", default_value="tb3_2"),
            DeclareLaunchArgument("second_x", default_value="1.0"),
            DeclareLaunchArgument("second_y", default_value="0.0"),
            DeclareLaunchArgument("second_yaw", default_value="0.0"),
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value=model),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch)),
            OpaqueFunction(function=_spawn_second_robot),
        ]
    )
