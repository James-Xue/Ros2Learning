#include "ros2_learning_cpp/listener.hpp"

ListenerNode::ListenerNode()
        : rclcpp::Node("listener")
{
    subscription_ = create_subscription<std_msgs::msg::String>(
        "chatter", 10, [this](const std_msgs::msg::String::SharedPtr msg)
        { RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str()); });

    RCLCPP_INFO(get_logger(), "listener started");
}
