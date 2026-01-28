#include "ros2_learning_panda_gazebo_demo/panda_joint_commander.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_learning_panda_gazebo_demo::PandaJointCommander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
