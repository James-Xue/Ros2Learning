#include "ros2_learning_behavior_tree/nodes/get_location_from_queue.hpp"
#include "rclcpp/rclcpp.hpp" // 引入 rclcpp

namespace ros2_learning_behavior_tree
{

GetLocationFromQueue::GetLocationFromQueue(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr) // 接收节点指针
: BT::SyncActionNode(name, config), node_(node_ptr)
{
  location_queue_ = {"Kitchen", "Bedroom", "Balcony", "Dock"};
}

BT::PortsList GetLocationFromQueue::providedPorts()
{
  return {
    BT::OutputPort<std::string>("target_location", "即将前往的目标点")
  };
}

BT::NodeStatus GetLocationFromQueue::tick()
{
  if (location_queue_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  std::string target = location_queue_[current_index_];
  current_index_ = (current_index_ + 1) % location_queue_.size();

  setOutput("target_location", target);

  // 改用 ROS 日志，且不加 [0m 这种控制符，由终端自己处理
  RCLCPP_INFO(node_->get_logger(), "[GetLocation] 🎲 从队列中取出目标: %s", target.c_str());

  return BT::NodeStatus::SUCCESS;
}

} // namespace ros2_learning_behavior_tree
