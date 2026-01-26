#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_turtlesim_go_to/srv/go_to.hpp"

namespace ros2_learning_turtlesim_go_to
{

class TurtlesimGoToClientNode : public rclcpp::Node
{
  public:
    explicit TurtlesimGoToClientNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    rclcpp::Client<srv::GoTo>::SharedFuture
    async_go_to(float x, float y, float theta, const std::string &turtle_name);

    std::chrono::duration<double> wait_service_timeout() const;
    std::chrono::duration<double> call_timeout() const;
    const std::string &service_name() const;

    rclcpp::Client<srv::GoTo>::SharedPtr client() const;

  private:
    std::string service_name_;
    double wait_service_timeout_sec_;
    double call_timeout_sec_;

    rclcpp::Client<srv::GoTo>::SharedPtr client_;
};

} // namespace ros2_learning_turtlesim_go_to
