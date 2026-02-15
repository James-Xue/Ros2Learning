#include "ros2_learning_multithreading/blocking_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * 该程序演示了如何根据参数选择不同的 ROS 2 执行器。
 * 我们可以通过命令行动态切换单线程或多线程模式，来观察 BlockingNode 的不同行为。
 */
int main(int argc, char * argv[])
{
    // 初始化 ROS 2 客户端库
    rclcpp::init(argc, argv);

    // 创建我们的演示节点实例
    // 该节点内含两个分别位于不同互斥回调组的定时器
    auto node = std::make_shared<ros2_learning_multithreading::BlockingNode>();

    // -------------------------------------------------------------------------
    // 知识点：动态参数
    // 我们定义一个参数 "executor_type"，允许在不重新编译的情况下切换执行器
    // 用法示例: ros2 run ros2_learning_multithreading executor_demo --ros-args -p executor_type:=multi
    // -------------------------------------------------------------------------
    node->declare_parameter("executor_type", "single");
    std::string executor_type = node->get_parameter("executor_type").as_string();

    RCLCPP_INFO(node->get_logger(), "当前执行器类型: %s", executor_type.c_str());

    // 根据参数值选择执行器逻辑
    if (executor_type == "multi")
    {
        // ---------------------------------------------------------------------
        // 知识点：MultiThreadedExecutor（多线程执行器）
        // 它会根据 CPU 核心数创建一个线程池（默认）。
        // 当有空闲线程且 CallbackGroup 允许时，它会并行运行回调。
        // ---------------------------------------------------------------------
        RCLCPP_INFO(node->get_logger(), "正在使用多线程执行器 [并行模式]");
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin(); // 进入事件循环，内部会分配多个工作线程
    }
    else
    {
        // ---------------------------------------------------------------------
        // 知识点：SingleThreadedExecutor（单线程执行器）
        // rclcpp::spin(node) 是最常见的简写形式，底层就是单线程执行器。
        // 它永远只使用当前这一个线程来处理所有事情，任务必须排队。
        // ---------------------------------------------------------------------
        RCLCPP_INFO(node->get_logger(), "正在使用单线程执行器 [串行阻塞模式]");
        rclcpp::spin(node); // 同步执行，这一个线程会处理所有回调
    }

    // 清理资源并退出
    rclcpp::shutdown();
    return 0;
}
