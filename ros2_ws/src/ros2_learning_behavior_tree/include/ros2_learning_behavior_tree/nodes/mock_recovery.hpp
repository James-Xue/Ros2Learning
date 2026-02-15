#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_behavior_tree
{

/**
 * @brief 模拟恢复行为节点 (MockRecovery)
 * 
 * 也是一个 StatefulActionNode。
 * 模拟出错后的恢复操作（如原地旋转、清除地图代价层）。
 */
class MockRecovery : public BT::StatefulActionNode
{
public:
  MockRecovery(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::chrono::system_clock::time_point start_time_;
  int duration_ms_ = 1000; // 默认耗时 1s
};

} // namespace ros2_learning_behavior_tree
