#ifndef ROS2_LEARNING_BEHAVIOR_TREE__NODES__SIMPLE_ARM_ACTION_HPP_
#define ROS2_LEARNING_BEHAVIOR_TREE__NODES__SIMPLE_ARM_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_behavior_tree
{

/**
 * @brief 简单的同步动作节点 (SyncActionNode)，模拟机械臂动作
 *
 * SyncActionNode 适用于那些“瞬间完成”或者执行时间极短的操作。
 *
 * 特点：
 * 1. 每次 tick() 必须立刻返回 SUCCESS 或 FAILURE。
 * 2. 严禁在此类节点中进行长时间的 sleep 或阻塞操作，否则会卡住整棵行为树的调度。
 */
class SimpleArmAction : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name 节点名
   * @param config 配置
   * @param node_ptr ROS 指针，用于对接系统日志
   */
  SimpleArmAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node_ptr);

  /**
   * @brief 端口列表
   * 包含一个输入参数：target_joint_angle (目标关节角度)
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 执行逻辑
   * 在此模拟发送机械臂控制指令并等待极短的时间。
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_; // ROS 节点指针
};

}  // namespace ros2_learning_behavior_tree

#endif  // ROS2_LEARNING_BEHAVIOR_TREE__NODES__SIMPLE_ARM_ACTION_HPP_
