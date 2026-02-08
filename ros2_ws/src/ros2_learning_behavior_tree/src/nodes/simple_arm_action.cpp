#include "ros2_learning_behavior_tree/nodes/simple_arm_action.hpp"

namespace ros2_learning_behavior_tree
{

SimpleArmAction::SimpleArmAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::SyncActionNode(name, config), node_(node_ptr)
{}

BT::PortsList SimpleArmAction::providedPorts()
{
  return {BT::InputPort<double>("target_joint_angle", "目标关节角度 (弧度)")};
}

BT::NodeStatus SimpleArmAction::tick()
{
  double angle = 0.0;
  if (!getInput<double>("target_joint_angle", angle)) {
    throw BT::RuntimeError("missing required input [target_joint_angle]");
  }

  RCLCPP_INFO(node_->get_logger(), "SimpleArmAction: 正在将机械臂移动到 %.2f (模拟中)...", angle);

  // 注意：这里使用 sleep 仅用于演示“同步阻塞”效果。
  // 在实际的 Production 代码中，如果动作超过几毫秒，不建议用 SyncActionNode + sleep，
  // 而应该用 StatefulActionNode + 异步，否则会卡住 BT 的 tick 循环，
  // 导致其他高优先级节点（如“急停监测”）无法及时响应。
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node_->get_logger(), "SimpleArmAction: 机械臂动作完成。");

  // 返回成功，树会继续执行 Sequence 中的下一个节点
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_learning_behavior_tree
