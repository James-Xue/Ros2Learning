#pragma once

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class Nav2Client : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using NavigateClient = rclcpp_action::Client<NavigateToPose>;
    using PosePublisher = rclcpp::Publisher<PoseWithCovarianceStamped>;

    Nav2Client();

    void run();

private:
    bool wait_for_server();
    void wait_for_time();
    void publish_initial_pose();
    GoalHandle::SharedPtr send_goal();

    PosePublisher::SharedPtr mInitialPosePublisher;
    NavigateClient::SharedPtr mActionClient;
};
