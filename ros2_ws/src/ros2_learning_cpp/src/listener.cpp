#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode final : public rclcpp::Node
{
public:
  ListenerNode()
  : rclcpp::Node("listener")
  {
    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
      });

    RCLCPP_INFO(get_logger(), "listener started");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
