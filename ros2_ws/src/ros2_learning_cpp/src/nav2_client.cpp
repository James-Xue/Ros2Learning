#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class Nav2Client : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Client()
  : Node("nav2_minimal_client")
  {
    this->declare_parameter<bool>("use_sim_time", true);
    this->set_parameter(rclcpp::Parameter("use_sim_time", this->get_parameter("use_sim_time").as_bool()));

    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void run()
  {
    wait_for_time();

    if (!wait_for_server()) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable.");
      return;
    }

    publish_initial_pose();

    auto goal_handle = send_goal();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Failed to send goal.");
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    auto result_code =
      rclcpp::spin_until_future_complete(shared_from_this(), result_future, 60s);

    if (result_code == rclcpp::FutureReturnCode::SUCCESS) {
      auto wrapped_result = result_future.get();
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Navigation succeeded.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(get_logger(), "Navigation aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "Navigation canceled.");
          break;
        default:
          RCLCPP_WARN(get_logger(), "Unknown result code.");
          break;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Navigation timeout, canceling goal.");
      auto cancel_future = action_client_->async_cancel_goal(goal_handle);
      rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
    }
  }

private:
  bool wait_for_server()
  {
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    return action_client_->wait_for_action_server(10s);
  }

  void wait_for_time()
  {
    while (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
      rclcpp::sleep_for(100ms);
    }
  }

  void publish_initial_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.25;

    RCLCPP_INFO(get_logger(), "Publishing initial pose to /initialpose");
    for (int i = 0; rclcpp::ok() && i < 5; ++i) {
      msg.header.stamp = this->now();
      initial_pose_pub_->publish(msg);
      rclcpp::sleep_for(200ms);
    }
  }

  GoalHandle::SharedPtr send_goal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    goal.pose.orientation = tf2::toMsg(q);

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(
          get_logger(),
          "Remaining distance: %.2f", feedback->distance_remaining);
      };

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(
        shared_from_this(), goal_handle_future, 5s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Send goal call failed");
      return nullptr;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      return nullptr;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted, waiting for result...");
    return goal_handle;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
