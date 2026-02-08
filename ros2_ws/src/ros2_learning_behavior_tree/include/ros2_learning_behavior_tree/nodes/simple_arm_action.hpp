#ifndef ROS2_LEARNING_BEHAVIOR_TREE__NODES__SIMPLE_ARM_ACTION_HPP_
#define ROS2_LEARNING_BEHAVIOR_TREE__NODES__SIMPLE_ARM_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_behavior_tree
{

/**
 * @brief 简单的同步动作节点 (SyncActionNode)，模拟机械臂动作
 *
 * SyncActionNode 适用于那些“瞬间完成”或者“时间极短且阻塞”的操作。
 *
 * 特点：
 * 1. 每次 tick() 必须立刻返回 SUCCESS 或 FAILURE。
 * 2. 如果你的任务需要 5 秒钟，在这个节点里 sleep 5 秒会 阻塞整棵树 的 tick！
 *    (对于长任务，请使用 StatefulActionNode，如 MoveBase)
 */
class SimpleArmAction : public BT::SyncActionNode
{
public:
  SimpleArmAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace ros2_learning_behavior_tree

#endif  // ROS2_LEARNING_BEHAVIOR_TREE__NODES__SIMPLE_ARM_ACTION_HPP_
