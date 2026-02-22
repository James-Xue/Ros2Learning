#ifndef ROS2_LEARNING_QOS_ROVER_NODE_HPP
#define ROS2_LEARNING_QOS_ROVER_NODE_HPP

#include <memory>

#include "rclcpp/node_options.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_learning_qos
{

/**
 * @brief 场景：火星探测车 (Mars Rover)
 * 职责：发布不同 QoS 策略的数据
 */
class RoverNode : public rclcpp::Node
{
public:
  explicit RoverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void publish_static_map();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;                // 定时器回调计数
  int skip_heartbeat_rounds_ = 0; // 跳过心跳的剩余轮数
};

} // namespace ros2_learning_qos

#endif // ROS2_LEARNING_QOS_ROVER_NODE_HPP
