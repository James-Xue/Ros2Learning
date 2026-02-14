#include "ros2_learning_cpp/listener.hpp"

namespace ros2_learning_cpp
{
    ListenerNode::ListenerNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("listener", options)
    {
        subscription_ = create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // 注意这里，如果走了 Intra-Process Comm，你会看到 Zero Copy 的效果
                RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
            });

        RCLCPP_INFO(get_logger(), "listener component started");
    }
}

// 注册 Listener 组件
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_cpp::ListenerNode)
