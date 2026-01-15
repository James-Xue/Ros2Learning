#include "ros2_learning_cpp/talker.hpp"

using namespace std::chrono_literals;

TalkerNode::TalkerNode()
        : rclcpp::Node("talker")
{
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(500ms, std::bind(&TalkerNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "talker started");
}

void TalkerNode::on_timer()
{
    std_msgs::msg::String msg;
    msg.data = "Hello ROS 2, count=" + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
}
