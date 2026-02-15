#include "ros2_learning_behavior_tree/nodes/mock_move_base.hpp"
#include <random>

namespace ros2_learning_behavior_tree
{

MockMoveBase::MockMoveBase(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
  // 初始化随机数生成器
  std::random_device rd;
  gen_ = std::mt19937(rd());
  dis_ = std::uniform_real_distribution<>(0.0, 1.0);
}

BT::PortsList MockMoveBase::providedPorts()
{
  return {
    BT::InputPort<std::string>("location", "Target A", "目标地点名称"),
    BT::InputPort<double>("probability", 1.0, "成功概率 (0.0 - 1.0)"),
    BT::InputPort<int>("duration", 2000, "模拟耗时 (ms)")
  };
}

BT::NodeStatus MockMoveBase::onStart()
{
  if (!getInput<std::string>("location", location_)) {
    throw BT::RuntimeError("missing location");
  }
  if (!getInput<double>("probability", probability_)) {
    throw BT::RuntimeError("missing probability");
  }
  if (!getInput<int>("duration", duration_ms_)) {
    throw BT::RuntimeError("missing duration");
  }

  start_time_ = std::chrono::system_clock::now();

  RCLCPP_INFO(node_->get_logger(), 
    "[MockMove] 🚀 前往 %s (耗时: %dms, 成功率: %.1f)...", 
    location_.c_str(), duration_ms_, probability_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MockMoveBase::onRunning()
{
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();

  if (elapsed < duration_ms_) {
    return BT::NodeStatus::RUNNING;
  }

  // 使用成员变量生成随机数
  double random_val = dis_(gen_);

  if (random_val <= probability_) {
    RCLCPP_INFO(node_->get_logger(), "[MockMove] ✅ 到达 %s 成功！", location_.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "[MockMove] ❌ 到达 %s 失败！(随机值: %.2f > 概率: %.2f)", 
      location_.c_str(), random_val, probability_);
    return BT::NodeStatus::FAILURE;
  }
}

void MockMoveBase::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[MockMove] 🛑 导航被中断！");
}

} // namespace ros2_learning_behavior_tree
