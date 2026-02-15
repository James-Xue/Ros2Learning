#include "ros2_learning_behavior_tree/nodes/mock_recovery.hpp"

namespace ros2_learning_behavior_tree
{

MockRecovery::MockRecovery(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
  // 此时节点已创建，但尚未开始执行任务。
}

BT::PortsList MockRecovery::providedPorts()
{
  // 定义该节点在 XML 中可以使用的参数（端口）
  return {
    BT::InputPort<std::string>("type", "Wait", "恢复类型名称 (如 Spin, ClearMap)"),
    BT::InputPort<int>("duration", 1000, "恢复耗时 (ms)") 
  };
}

BT::NodeStatus MockRecovery::onStart()
{
  // 1. 获取输入参数
  std::string type;
  if (!getInput("type", type)) {
      type = "Unknown";
  }
  
  // 尝试读取可选参数 duration，如果 XML 中没配置，则保持初始值 (1000)
  int duration = 1000;
  if (getInput("duration", duration)) {
      duration_ms_ = duration;
  }

  // 2. 记录当前时间作为起点
  start_time_ = std::chrono::system_clock::now();

  // 3. 打印日志通知用户
  RCLCPP_WARN(node_->get_logger(), "[Recovery] ⚠️  开始恢复行为: %s (耗时 %dms)...", type.c_str(), duration_ms_);

  // 4. 返回 RUNNING，告知行为树调度器进入异步运行阶段
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MockRecovery::onRunning()
{
  // 查看从开始到现在经过了多少毫秒
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();

  // 如果还没到预设时间，继续返回 RUNNING
  if (elapsed < duration_ms_) {
    return BT::NodeStatus::RUNNING;
  }

  // 时间已到，恢复完成
  RCLCPP_INFO(node_->get_logger(), "[Recovery] ✨ 恢复完成，机器人已复位。");
  return BT::NodeStatus::SUCCESS;
}

void MockRecovery::onHalted()
{
  // 如果任务在中途被父节点强行终止，打印该信息。
  // 在真实场景下，你可能需要在这里发送停止指令给底层电机。
  RCLCPP_INFO(node_->get_logger(), "[Recovery] 🛑 恢复动作被外部中断！");
}

} // namespace ros2_learning_behavior_tree
