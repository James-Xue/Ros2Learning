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
        // 类型别名，简化代码
        using SubscriptionPtr = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;

        explicit ListenerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        SubscriptionPtr subscription_;
    };
}
