#include "ros2_learning_behavior_tree/nodes/move_base_node.hpp"

namespace ros2_learning_behavior_tree
{

MoveBase::MoveBase(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
  // 构造函数：创建 ROS 2 Action Client 实例
  // 必须确保服务器名称 ("navigate_to_pose") 与 Nav2 实际提供的名称一致
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList MoveBase::providedPorts()
{
  // 定义输入端口，用于从行为树黑板获取目标坐标
  return {
    BT::InputPort<double>("goal_x", "目标点的 X 坐标 (米)"),
    BT::InputPort<double>("goal_y", "目标点的 Y 坐标 (米)"),
    BT::InputPort<double>("goal_yaw", "目标点的偏航角 (弧度)")
  };
}

BT::NodeStatus MoveBase::onStart()
{
  // 1. 检查动作服务器是否在线
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "MoveBase: Action 服务器不可用 (Nav2 可能未启动)");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 从端口读取坐标
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!getInput<double>("goal_x", x)) {
    throw BT::RuntimeError("MoveBase: 缺少必填参数 [goal_x]");
  }
  if (!getInput<double>("goal_y", y)) {
    throw BT::RuntimeError("MoveBase: 缺少必填参数 [goal_y]");
  }
  if (!getInput<double>("goal_yaw", yaw)) {
    throw BT::RuntimeError("MoveBase: 缺少必填参数 [goal_yaw]");
  }

  RCLCPP_INFO(node_->get_logger(), "MoveBase: 向 Nav2 发送目标 (%.2f, %.2f, %.2f)", x, y, yaw);

  // 3. 构建 Nav2 Action Goal 消息
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";           // 基准坐标系
  goal_msg.pose.header.stamp = node_->now();        // 当前 ROS 时间戳
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;

  // 将欧拉角转化为四元数（ROS 姿态的标准表示）
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // 4. 设置异步回调并发送目标
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  // 当导航完成后，ROS 会调用这个 result_callback
  send_goal_options.result_callback =
      std::bind(&MoveBase::result_callback, this, std::placeholders::_1);

  // 注意：我们使用的是异步发送，不会阻塞行为树的 Tick 循环
  future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);

  // 返回 RUNNING，告知调度器任务进入异步执行状态
  return BT::NodeStatus::RUNNING;
}

void MoveBase::result_callback(const GoalHandleNav::WrappedResult & result)
{
  // 异步回调：当 Nav2 结束任务时触发
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "MoveBase: [核准] 机器人顺利到达目标位置！");
    nav_result_status_ = BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(node_->get_logger(), "MoveBase: [异常] 导航任务被中止 (Aborted)");
    nav_result_status_ = BT::NodeStatus::FAILURE;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(node_->get_logger(), "MoveBase: [用户动作] 导航任务已被取消");
    nav_result_status_ = BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "MoveBase: [错误] 由于未知原因导航失败");
    nav_result_status_ = BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveBase::onRunning()
{
  // 1. 检查服务器是否已经接受了我们发送的目标
  if (future_goal_handle_.valid()) {
    auto status = future_goal_handle_.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      goal_handle_ = future_goal_handle_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "MoveBase: 目标被服务器拒绝响应 (Rejected)");
        return BT::NodeStatus::FAILURE;
      } else {
        RCLCPP_INFO(node_->get_logger(), "MoveBase: Nav2 服务器已接受任务并由于正在执行...");
      }
    }
  }

  // 2. 检查异步结果是否已经准备好
  if (nav_result_status_.has_value()) {
    auto status = nav_result_status_.value();
    
    // 清理状态，为下次可能的重新执行做准备
    nav_result_status_.reset();
    goal_handle_.reset();
    return status; // 返回最终的 SUCCESS 或 FAILURE
  }

  // 任务仍在进行中
  return BT::NodeStatus::RUNNING;
}

void MoveBase::onHalted()
{
  // 抢占处理：如果有更高优先级的节点打断了当前导航
  RCLCPP_INFO(node_->get_logger(), "MoveBase: 收到中断 (Halt) 信号，正在停止导航并清理资源");
  if (goal_handle_) {
    // 必须向 Nav2 服务器发送取消指令，否则机器人会继续移动！
    action_client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace ros2_learning_behavior_tree
