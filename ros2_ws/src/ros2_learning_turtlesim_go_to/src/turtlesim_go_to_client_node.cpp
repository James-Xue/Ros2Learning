#include "ros2_learning_turtlesim_go_to/turtlesim_go_to_client_node.hpp"

namespace ros2_learning_turtlesim_go_to
{

TurtlesimGoToClientNode::TurtlesimGoToClientNode(
    const rclcpp::NodeOptions &options)
        : rclcpp::Node("turtlesim_go_to_client", options)
{
    service_name_ = this->declare_parameter<std::string>(
        "service_name", "/ros2_learning/turtlesim/go_to");

    wait_service_timeout_sec_ =
        this->declare_parameter<double>("wait_service_timeout_sec", 2.0);

    call_timeout_sec_ =
        this->declare_parameter<double>("call_timeout_sec", 2.0);

    client_ = this->create_client<srv::GoTo>(service_name_);
}

rclcpp::Client<srv::GoTo>::SharedFuture
TurtlesimGoToClientNode::async_go_to(float x, float y, float theta,
                                     const std::string &turtle_name)
{
    auto req = std::make_shared<srv::GoTo::Request>();
    req->turtle_name = turtle_name;
    req->x = x;
    req->y = y;
    req->theta = theta;
    return client_->async_send_request(req).future.share();
}

std::chrono::duration<double>
TurtlesimGoToClientNode::wait_service_timeout() const
{
    return std::chrono::duration<double>(wait_service_timeout_sec_);
}

std::chrono::duration<double> TurtlesimGoToClientNode::call_timeout() const
{
    return std::chrono::duration<double>(call_timeout_sec_);
}

const std::string &TurtlesimGoToClientNode::service_name() const
{
    return service_name_;
}

rclcpp::Client<srv::GoTo>::SharedPtr TurtlesimGoToClientNode::client() const
{
    return client_;
}

} // namespace ros2_learning_turtlesim_go_to
