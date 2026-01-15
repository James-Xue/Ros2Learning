#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node
{
  public:
    ListenerNode();

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
