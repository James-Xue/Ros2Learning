#include <memory>

#include "nav2_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Client>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
