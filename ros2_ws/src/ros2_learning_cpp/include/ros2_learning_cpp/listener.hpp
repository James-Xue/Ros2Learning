#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_learning_cpp
{
    class ListenerNode : public rclcpp::Node
    {
    public:
        explicit ListenerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };
}
