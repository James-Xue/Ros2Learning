#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_turtlesim_go_to/turtlesim_go_to_client_node.hpp"

namespace
{
void print_usage(const char *prog)
{
    std::cerr << "Usage:\n  " << prog << " <x> <y> <theta> [turtle_name]\n\n"
              << "Example:\n  " << prog << " 5.5 5.5 1.57 turtle1\n\n"
              << "Notes:\n"
              << "- This client calls service: /ros2_learning/turtlesim/go_to\n"
              << "- Server will drive turtlesim smoothly via: "
                 "/<turtle_name>/cmd_vel\n";
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

    auto node = std::make_shared<
        ros2_learning_turtlesim_go_to::TurtlesimGoToClientNode>();

    if (!node->client()->wait_for_service(node->wait_service_timeout()))
    {
        std::cerr << "Service not available: " << node->service_name() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    auto future =
        node->async_go_to(static_cast<float>(x), static_cast<float>(y),
                          static_cast<float>(theta), turtle_name);

    const auto rc =
        rclcpp::spin_until_future_complete(node, future, node->call_timeout());
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cerr << "Failed waiting for response (rc=" << static_cast<int>(rc)
                  << ").\n";
        rclcpp::shutdown();
        return 1;
    }

    const auto resp = future.get();
    std::cout << (resp->success ? "SUCCESS" : "FAIL") << ": " << resp->message
              << "\n";

    rclcpp::shutdown();
    return resp->success ? 0 : 1;
}
