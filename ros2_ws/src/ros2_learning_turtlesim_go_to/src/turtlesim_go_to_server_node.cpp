#include "ros2_learning_turtlesim_go_to/turtlesim_go_to_server_node.hpp"

#include <chrono>
#include <sstream>

namespace ros2_learning_turtlesim_go_to
{

using namespace std::chrono_literals;

TurtlesimGoToServerNode::TurtlesimGoToServerNode()
        : rclcpp::Node("turtlesim_go_to_server")
{
    go_to_service_name_ = this->declare_parameter<std::string>(
        "service_name", "/ros2_learning/turtlesim/go_to");

    turtlesim_wait_timeout_sec_ =
        this->declare_parameter<double>("turtlesim_wait_timeout_sec", 2.0);

    turtlesim_call_timeout_sec_ =
        this->declare_parameter<double>("turtlesim_call_timeout_sec", 2.0);

    service_ = this->create_service<GoTo>(
        go_to_service_name_,
        std::bind(&TurtlesimGoToServerNode::handle_go_to, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "GoTo service ready: %s",
                go_to_service_name_.c_str());
}

void TurtlesimGoToServerNode::handle_go_to(
    const std::shared_ptr<GoTo::Request> request,
    std::shared_ptr<GoTo::Response> response)
{
    // This callback executes in the executor thread.
    // We'll synchronously wait for turtlesim response. Using a MultiThreaded
    // executor in main avoids deadlocks when waiting.

    const std::string turtle =
        request->turtle_name.empty() ? "turtle1" : request->turtle_name;

    const std::string teleport_srv = "/" + turtle + "/teleport_absolute";

    auto client = this->create_client<TeleportAbsolute>(teleport_srv);

    const auto wait_timeout =
        std::chrono::duration<double>(turtlesim_wait_timeout_sec_);
    if (!client->wait_for_service(wait_timeout))
    {
        response->success = false;
        response->message =
            "turtlesim service not available: " + teleport_srv +
            ". Did you run: ros2 run turtlesim turtlesim_node ?";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    auto req = std::make_shared<TeleportAbsolute::Request>();
    req->x = request->x;
    req->y = request->y;
    req->theta = request->theta;

    RCLCPP_INFO(get_logger(), "Teleporting %s to (%.3f, %.3f, %.3f)",
                turtle.c_str(), request->x, request->y, request->theta);

    auto future = client->async_send_request(req);

    const auto call_timeout =
        std::chrono::duration<double>(turtlesim_call_timeout_sec_);
    const auto status = future.wait_for(call_timeout);
    if (status != std::future_status::ready)
    {
        response->success = false;
        response->message = "timeout calling " + teleport_srv;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    // turtlesim TeleportAbsolute response is empty; if we get here, call
    // worked.
    (void)future.get();

    response->success = true;
    std::ostringstream oss;
    oss << "ok: " << turtle << " => (" << request->x << ", " << request->y
        << ", " << request->theta << ")";
    response->message = oss.str();
}

} // namespace ros2_learning_turtlesim_go_to
