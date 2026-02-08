#ifndef ROS2_LEARNING_BEHAVIOR_TREE__NODES__MOVE_BASE_NODE_HPP_
#define ROS2_LEARNING_BEHAVIOR_TREE__NODES__MOVE_BASE_NODE_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_learning_behavior_tree
{

/**
 * @brief 封装 Nav2 NavigateToPose 的同步/异步 Action 节点
 *
 * 这是一个 StatefulActionNode，意味着它可以返回 RUNNING 状态。
 * 在 BT 中，当它处于 RUNNING 时，树的其他部分通常会等待它完成（取决于是否在 Sequence 中）。
 *
 * 知识点：
 * 1. StatefulActionNode vs SyncActionNode:
 *    - SyncActionNode: 一次 tick 必须返回 SUCCESS 或 FAILURE，不能阻塞太久。
 *    - StatefulActionNode: 可以返回 RUNNING，告诉树“我正在干活，还没完”。适合耗时任务（如导航）。
 *
 * 2. 异步 Action Client:
 *    - 使用 rclcpp_action::Client 非阻塞地发送目标。
 *    - 在回调中更新状态。
 */
class MoveBase : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MoveBase(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_;
  GoalHandleNav::SharedPtr goal_handle_;

  std::optional<BT::NodeStatus> nav_result_status_;
};

}  // namespace ros2_learning_behavior_tree

#endif  // ROS2_LEARNING_BEHAVIOR_TREE__NODES__MOVE_BASE_NODE_HPP_
