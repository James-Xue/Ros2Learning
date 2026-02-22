#ifndef ROS2_LEARNING_QOS_MISSION_CONTROL_NODE_HPP
#define ROS2_LEARNING_QOS_MISSION_CONTROL_NODE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_learning_qos
{

/**
 * @brief 场景：地球控制中心 (Mission Control)
 * 职责：订阅数据，监控 QoS 事件
 */
class MissionControlNode : public rclcpp::Node
{
public:
  MissionControlNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr incompatible_sub_;
};

} // namespace ros2_learning_qos

#endif // ROS2_LEARNING_QOS_MISSION_CONTROL_NODE_HPP
