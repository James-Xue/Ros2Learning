#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <random> // 引入 random

namespace ros2_learning_behavior_tree
{

/**
 * @brief 模拟移动节点 (MockMoveBase)
 * 
 * 这是一个 StatefulActionNode（异步动作节点）。
 * 它模拟一个耗时的导航过程，并根据设定的概率随机成功或失败。
 */
class MockMoveBase : public BT::StatefulActionNode
{
public:
  MockMoveBase(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::chrono::system_clock::time_point start_time_;
  int duration_ms_;
  double probability_;
  std::string location_;

  // 将随机数生成器作为成员变量，避免 static 的多线程风险
  std::mt19937 gen_;
  std::uniform_real_distribution<> dis_;
};

} // namespace ros2_learning_behavior_tree
