#include <memory>

#include "ros2_learning_cpp/talker.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto pNode = std::make_shared<TalkerNode>();
    RCLCPP_INFO(pNode->get_logger(), " 你好 C++ 节点！ ");

    rclcpp::spin(pNode);
    rclcpp::shutdown();
    return 0;
}
