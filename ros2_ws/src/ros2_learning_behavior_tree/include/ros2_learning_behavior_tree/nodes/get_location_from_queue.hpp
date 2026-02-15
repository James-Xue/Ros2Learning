#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp" // 补上这个
#include <vector>
#include <string>

namespace ros2_learning_behavior_tree
{

/**
 * @brief 队列取点节点 (GetLocationFromQueue)
 * 
 * 这是一个同步节点（瞬间完成）。
 * 它内部维护一个目标点队列，每次被调用时取出一个点，写入 Blackboard。
 */
class GetLocationFromQueue : public BT::SyncActionNode
{
public:
  // 修改构造函数，接收 ROS 节点以打印日志
  GetLocationFromQueue(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_; // 新增
  std::vector<std::string> location_queue_;
  size_t current_index_ = 0;
};

} // namespace ros2_learning_behavior_tree
