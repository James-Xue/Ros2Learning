#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_tb3_spawner/tb3_spawner_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ros2_learning_tb3_spawner::Tb3SpawnerNode>();

    // We may block inside service callback while waiting for Gazebo's response,
    // so use a MultiThreadedExecutor to keep processing incoming responses.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
