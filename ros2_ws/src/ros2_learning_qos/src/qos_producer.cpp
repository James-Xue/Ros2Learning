#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_qos/qos_example_nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ros2_learning_qos::RoverNode>();
  
  RCLCPP_INFO(node->get_logger(), "======== [Mars Rover] QoS Producer Started ========");
  RCLCPP_INFO(node->get_logger(), "  - Video Stream: Best Effort (Volatile)");
  RCLCPP_INFO(node->get_logger(), "  - Command Ack:  Reliable (Volatile)");
  RCLCPP_INFO(node->get_logger(), "  - Map Data:     Reliable (Transient Local - Latched)");
  RCLCPP_INFO(node->get_logger(), "  - Heartbeat:    Deadline (600ms)");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
