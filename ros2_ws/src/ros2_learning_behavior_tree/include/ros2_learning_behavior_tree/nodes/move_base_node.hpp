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
 * @brief MoveBase 行为树节点 (真实 Nav2 客户端)
 * 
 * 这是一个 StatefulActionNode (动作节点)，作为桥梁连接行为树与 ROS 2 的 Navigation 2 栈。
 * 它通过异步 Action Client 发送目标位姿，并在任务期间返回 RUNNING 状态。
 *
 * 知识点：
 * 1. StatefulActionNode vs SyncActionNode:
 *    - SyncActionNode: 一次 tick 必须返回 SUCCESS 或 FAILURE，适合快速任务。
 *    - StatefulActionNode: 可以返回 RUNNING，适合此类耗时且有状态的导航任务。
 *
 * 2. 异步 Action Client:
 *    - 使用 rclcpp_action::Client 向 Nav2 节点发送目标，避免阻塞 BT 调度。
 */
class MoveBase : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief 构造函数
   * @param name 节点实例名
   * @param config 节点配置
   * @param node_ptr ROS 节点指针，用于创建 Action Client
   */
  MoveBase(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node_ptr);

  /**
   * @brief 定义端口列表
   * 提供三个输入端口：goal_x, goal_y, goal_yaw
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 动作启动钩子
   * 在这里读取黑板数据并异步发送 Action Goal。
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief 动作运行钩子
   * 轮询异步任务的状态，并通知行为树。
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief 动作中断钩子
   * 如果由于某种原因导航被取消，发送 Cancel Goal 给 Nav2 服务器。
   */
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;                                     // ROS 节点指针
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;    // Nav2 动作客户端

  std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_; // 异步发送目标后的 Future
  GoalHandleNav::SharedPtr goal_handle_;                             // 目标句柄，用于取消或检查状态

  std::optional<BT::NodeStatus> nav_result_status_;                 // 延迟存储导航结果，直到下一轮 Tick

  /**
   * @brief 处理导航结果的回调函数
   * @param result 包裹后的动作结果回执
   */
  void result_callback(const GoalHandleNav::WrappedResult & result);
};

}  // namespace ros2_learning_behavior_tree

#endif  // ROS2_LEARNING_BEHAVIOR_TREE__NODES__MOVE_BASE_NODE_HPP_
