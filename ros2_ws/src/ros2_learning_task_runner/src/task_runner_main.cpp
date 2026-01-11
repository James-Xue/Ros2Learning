#include "task_runner.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<TaskRunner>();
        node->run();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "[task_runner] FATAL: exception: %s\n", e.what());
        rclcpp::shutdown();
        return 2;
    }
}
