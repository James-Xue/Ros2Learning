#include "ros2_learning_behavior_tree/nodes/mock_recovery.hpp"

namespace ros2_learning_behavior_tree
{

MockRecovery::MockRecovery(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
}

BT::PortsList MockRecovery::providedPorts()
{
  return {
    BT::InputPort<std::string>("type", "Wait", "恢复类型名称"),
    BT::InputPort<int>("duration", 1000, "恢复耗时 (ms)") // 补上这个端口声明
  };
}

BT::NodeStatus MockRecovery::onStart()
{
  std::string type;
  getInput("type", type);
  
  // 尝试读取可选参数 duration，如果没设置，默认为 1000
  // 注意：虽然我们在 .hpp 里给了 duration_ms_ 初始值 1000，
  // 但如果想支持从 XML 配置，最好在这里 getInput 一下。
  // 不过为了简化，我们假设它主要由 XML 属性控制。
  // 我们在 XML 里没写 duration，所以它会用默认值。
  // 这里我们加上对 duration 的读取，如果 XML 里有，就覆盖。
  int duration = 1000;
  if (getInput("duration", duration)) {
      duration_ms_ = duration;
  }

  start_time_ = std::chrono::system_clock::now();
  RCLCPP_WARN(node_->get_logger(), "[Recovery] ⚠️  开始恢复行为: %s (耗时 %dms)...", type.c_str(), duration_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MockRecovery::onRunning()
{
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();

  // 使用成员变量，而不是硬编码的 1000
  if (elapsed < duration_ms_) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "[Recovery] ✨ 恢复完成，机器人已复位。");
  return BT::NodeStatus::SUCCESS;
}

void MockRecovery::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[Recovery] 🛑 恢复被中断！");
}

} // namespace ros2_learning_behavior_tree
