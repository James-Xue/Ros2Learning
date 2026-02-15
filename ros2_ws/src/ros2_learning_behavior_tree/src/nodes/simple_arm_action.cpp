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
  // 定义输入端口，获取机械臂需要转动的目标角度
  return {BT::InputPort<double>("target_joint_angle", "目标关节角度 (弧度)")};
}

BT::NodeStatus SimpleArmAction::tick()
{
  // 1. 获取输入参数
  double angle = 0.0;
  if (!getInput<double>("target_joint_angle", angle)) {
    throw BT::RuntimeError("SimpleArmAction: 缺少必填参数 [target_joint_angle]");
  }

  // 2. 模拟执行动作
  RCLCPP_INFO(node_->get_logger(), "SimpleArmAction: 🚀 正在将机械臂移动到角度 %.2f (模拟执行中)...", angle);

  // -------------------------------------------------------------------------
  // 知识点：同步节点的阻塞风险
  // 警告：这里使用了 sleep_for 模拟 1 秒的执行时间。
  // 在实际项目代码中，如果动作超过几毫秒且不可预测，绝对不建议在 SyncActionNode 中阻塞！
  // 这样做会导致整棵行为树在这 1 秒内完全“静止”，无法检查任何更高优先级的条件。
  // 这种情况下，推荐升级为 StatefulActionNode 并采用异步模式。
  // -------------------------------------------------------------------------
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node_->get_logger(), "SimpleArmAction: ✅ 机械臂动作已完成。");

  // 3. 返回执行成功，由于是 Sequence 的子节点，控制权将交给下一个节点
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_learning_behavior_tree
