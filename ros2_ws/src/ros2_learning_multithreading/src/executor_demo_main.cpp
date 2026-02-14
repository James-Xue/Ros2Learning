#include "ros2_learning_multithreading/blocking_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ros2_learning_multithreading::BlockingNode>();

    // 检查命令行参数
    // 用法: ros2 run ... --ros-args -p executor_type:=multi
    node->declare_parameter("executor_type", "single");
    std::string executor_type = node->get_parameter("executor_type").as_string();

    RCLCPP_INFO(node->get_logger(), "Executor Type: %s", executor_type.c_str());

    if (executor_type == "multi")
    {
        RCLCPP_INFO(node->get_logger(), "Using MultiThreadedExecutor (Parallel!)");
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Using SingleThreadedExecutor (Blocking!)");
        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}
