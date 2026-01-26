#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_turtlesim_go_to/turtlesim_go_to_server_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<
        ros2_learning_turtlesim_go_to::TurtlesimGoToServerNode>();

    // Service callback blocks waiting for turtlesim's response.
    // MultiThreadedExecutor helps avoid deadlocks during synchronous waits.
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
