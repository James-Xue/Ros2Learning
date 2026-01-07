#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_nav2_client/nav2_client.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Client>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
