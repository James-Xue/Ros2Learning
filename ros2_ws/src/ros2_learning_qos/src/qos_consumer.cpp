#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_qos/qos_example_nodes.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ros2_learning_qos::MissionControlNode>();
  
  RCLCPP_INFO(node->get_logger(), "======== [Mission Control] QoS Consumer Started ========");
  RCLCPP_INFO(node->get_logger(), "Waiting for data from Mars Rover...");
  RCLCPP_INFO(node->get_logger(), "QoS Policies:");
  RCLCPP_INFO(node->get_logger(), "  - Sub 1 (Video): Best Effort");
  RCLCPP_INFO(node->get_logger(), "  - Sub 2 (Ack):   Reliable");
  RCLCPP_INFO(node->get_logger(), "  - Sub 3 (Map):   Transient Local (Should receive history)");
  RCLCPP_INFO(node->get_logger(), "  - Sub 4 (Heartbeat): Deadline (Alarm if missed)");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
