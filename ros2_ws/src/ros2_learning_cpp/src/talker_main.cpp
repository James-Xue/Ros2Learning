#include <memory>

#include "talker.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}
