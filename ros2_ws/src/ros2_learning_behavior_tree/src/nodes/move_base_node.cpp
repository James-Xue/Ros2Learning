#include "ros2_learning_behavior_tree/nodes/move_base_node.hpp"

namespace ros2_learning_behavior_tree
{

MoveBase::MoveBase(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList MoveBase::providedPorts()
{
  return {
    BT::InputPort<double>("goal_x", "目标点的 X 坐标"),
    BT::InputPort<double>("goal_y", "目标点的 Y 坐标"),
    BT::InputPort<double>("goal_yaw", "目标点的偏航角 (弧度)")
  };
}

BT::NodeStatus MoveBase::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "MoveBase: Action server not avaiable (Nav2 没启动?)");
    return BT::NodeStatus::FAILURE;
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!getInput<double>("goal_x", x)) {
    throw BT::RuntimeError("missing required input [goal_x]");
  }
  if (!getInput<double>("goal_y", y)) {
    throw BT::RuntimeError("missing required input [goal_y]");
  }
  if (!getInput<double>("goal_yaw", yaw)) {
    throw BT::RuntimeError("missing required input [goal_yaw]");
  }

  RCLCPP_INFO(node_->get_logger(), "MoveBase: 发送导航目标 (%.2f, %.2f, %.2f)", x, y, yaw);

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "MoveBase: 导航成功到达！");
        nav_result_status_ = BT::NodeStatus::SUCCESS;
      } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_WARN(node_->get_logger(), "MoveBase: 导航被中止 (Aborted)");
        nav_result_status_ = BT::NodeStatus::FAILURE;
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "MoveBase: 导航被取消 (Canceled)");
        nav_result_status_ = BT::NodeStatus::FAILURE;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "MoveBase: 导航失败 (Unknown)");
        nav_result_status_ = BT::NodeStatus::FAILURE;
      }
    };

  future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBase::onRunning()
{
  if (future_goal_handle_.valid()) {
    auto status = future_goal_handle_.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      goal_handle_ = future_goal_handle_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "MoveBase: 目标被服务器拒绝了 (Rejected)");
        return BT::NodeStatus::FAILURE;
      } else {
        RCLCPP_INFO(node_->get_logger(), "MoveBase: 服务器已接收目标，正在执行...");
      }
    }
  }

  if (nav_result_status_.has_value()) {
    auto status = nav_result_status_.value();
    nav_result_status_.reset();
    goal_handle_.reset();
    return status;
  }

  return BT::NodeStatus::RUNNING;
}

void MoveBase::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "MoveBase: 收到 Halt 信号，取消导航");
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace ros2_learning_behavior_tree
