#include "ros2_learning_cpp/nav2_client.hpp"

using namespace std::chrono_literals;

Nav2Client::Nav2Client()
    : Node("nav2_minimal_client")
{
    this->declare_parameter<bool>("use_sim_time", true);
    this->set_parameter(rclcpp::Parameter("use_sim_time", this->get_parameter("use_sim_time").as_bool()));

    mInitialPosePublisher = this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);
    mActionClient = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

void Nav2Client::run()
{
    wait_for_time();

    if (!wait_for_server())
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable.");
        return;
    }

    publish_initial_pose();

    auto goal_handle = send_goal();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send goal.");
        return;
    }

    auto result_future = mActionClient->async_get_result(goal_handle);
    auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), result_future, 60s);

    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto wrapped_result = result_future.get();
        switch (wrapped_result.code)
        {
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
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Navigation timeout, canceling goal.");
        auto cancel_future = mActionClient->async_cancel_goal(goal_handle);
        rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
    }
}

bool Nav2Client::wait_for_server()
{
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    return mActionClient->wait_for_action_server(10s);
}

void Nav2Client::wait_for_time()
{
    while (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0)
    {
        rclcpp::sleep_for(100ms);
    }
}

void Nav2Client::publish_initial_pose()
{
    PoseWithCovarianceStamped msg;
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
    for (int i = 0; rclcpp::ok() && i < 5; ++i)
    {
        msg.header.stamp = this->now();
        mInitialPosePublisher->publish(msg);
        rclcpp::sleep_for(200ms);
    }
}

Nav2Client::GoalHandle::SharedPtr Nav2Client::send_goal()
{
    PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    goal.pose.orientation = tf2::toMsg(q);

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal;

    auto send_goal_options = NavigateClient::SendGoalOptions();
    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        { RCLCPP_INFO(get_logger(), "Remaining distance: %.2f", feedback->distance_remaining); };

    auto goal_handle_future = mActionClient->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Send goal call failed");
        return nullptr;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        return nullptr;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted, waiting for result...");
    return goal_handle;
}
