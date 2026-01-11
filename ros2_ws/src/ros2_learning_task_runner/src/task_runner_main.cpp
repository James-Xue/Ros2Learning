// task_runner 节点入口：初始化 ROS2，运行任务编排
#include "task_runner.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    try
    {
        // 创建节点并运行主流程
        auto node = std::make_shared<TaskRunner>();
        node->run();
        // 正常结束
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        // 捕获异常避免静默退出
        fprintf(stderr, "[task_runner] FATAL: exception: %s\n", e.what());
        rclcpp::shutdown();
        return 2;
    }
}
