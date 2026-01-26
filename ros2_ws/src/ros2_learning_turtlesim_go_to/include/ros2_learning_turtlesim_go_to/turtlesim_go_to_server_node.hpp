#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "ros2_learning_turtlesim_go_to/srv/go_to.hpp"
#include "turtlesim/msg/pose.hpp"

namespace ros2_learning_turtlesim_go_to
{

class TurtlesimGoToServerNode : public rclcpp::Node
{
  public:
    TurtlesimGoToServerNode();

  private:
    using GoTo = ros2_learning_turtlesim_go_to::srv::GoTo;
    using Pose = turtlesim::msg::Pose;
    using Twist = geometry_msgs::msg::Twist;

    void handle_go_to(const std::shared_ptr<GoTo::Request> request,
                      std::shared_ptr<GoTo::Response> response);

    void ensure_io_for_turtle(const std::string &turtle);

    std::string go_to_service_name_;

    // Controller parameters
    double k_linear_;
    double k_angular_;
    double max_linear_;
    double max_angular_;
    double dist_tolerance_;
    double theta_tolerance_;
    double control_rate_hz_;
    double wait_pose_timeout_sec_;
    double overall_timeout_sec_;

    // Current turtle IO
    std::string current_turtle_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

    std::mutex pose_mutex_;
    Pose last_pose_{};
    std::atomic<bool> pose_received_{false};

    std::atomic<bool> busy_{false};

    rclcpp::Service<GoTo>::SharedPtr service_;
};

} // namespace ros2_learning_turtlesim_go_to
