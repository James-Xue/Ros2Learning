#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_turtlesim_go_to/srv/go_to.hpp"

namespace
{
void print_usage(const char *prog)
{
    std::cerr << "Usage:\n  " << prog << " <x> <y> <theta> [turtle_name]\n\n"
              << "Example:\n  " << prog << " 5.5 5.5 1.57 turtle1\n\n"
              << "Notes:\n"
              << "- This client calls service: /ros2_learning/turtlesim/go_to\n"
              << "- Server will forward to turtlesim: "
                 "/<turtle_name>/teleport_absolute\n";
}

bool parse_double(const char *s, double &out)
{
    char *end = nullptr;
    const double v = std::strtod(s, &end);
    if (!end || end == s || *end != '\0')
    {
        return false;
    }
    out = v;
    return true;
}
} // namespace

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        print_usage(argv[0]);
        return 2;
    }

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    if (!parse_double(argv[1], x) || !parse_double(argv[2], y) ||
        !parse_double(argv[3], theta))
    {
        std::cerr << "Invalid numeric arguments.\n";
        print_usage(argv[0]);
        return 2;
    }

    const std::string turtle_name = (argc >= 5) ? argv[4] : "turtle1";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("turtlesim_go_to_client");

    using GoTo = ros2_learning_turtlesim_go_to::srv::GoTo;
    auto client = node->create_client<GoTo>("/ros2_learning/turtlesim/go_to");

    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
        std::cerr << "Service not available: /ros2_learning/turtlesim/go_to\n";
        rclcpp::shutdown();
        return 1;
    }

    auto req = std::make_shared<GoTo::Request>();
    req->turtle_name = turtle_name;
    req->x = static_cast<float>(x);
    req->y = static_cast<float>(y);
    req->theta = static_cast<float>(theta);

    auto future = client->async_send_request(req);

    const auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready)
    {
        std::cerr << "Timed out waiting for response.\n";
        rclcpp::shutdown();
        return 1;
    }

    const auto resp = future.get();
    std::cout << (resp->success ? "SUCCESS" : "FAIL") << ": " << resp->message
              << "\n";

    rclcpp::shutdown();
    return resp->success ? 0 : 1;
}
