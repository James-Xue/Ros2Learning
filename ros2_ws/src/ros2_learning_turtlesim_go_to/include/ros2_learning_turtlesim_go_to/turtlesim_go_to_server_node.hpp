#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_turtlesim_go_to/srv/go_to.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

namespace ros2_learning_turtlesim_go_to
{

class TurtlesimGoToServerNode : public rclcpp::Node
{
  public:
    TurtlesimGoToServerNode();

  private:
    using GoTo = ros2_learning_turtlesim_go_to::srv::GoTo;
    using TeleportAbsolute = turtlesim::srv::TeleportAbsolute;

    void handle_go_to(const std::shared_ptr<GoTo::Request> request,
                      std::shared_ptr<GoTo::Response> response);

    std::string go_to_service_name_;
    double turtlesim_wait_timeout_sec_;
    double turtlesim_call_timeout_sec_;

    rclcpp::Service<GoTo>::SharedPtr service_;
};

} // namespace ros2_learning_turtlesim_go_to
