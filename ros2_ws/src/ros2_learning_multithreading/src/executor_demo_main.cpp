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
    // 知识点：硬件级看门狗往往是一个独立的硬件或最高优先级的独立线程
    // 为了在这个 Demo 中让看门狗能监控 "单线程执行器被卡死" 的情况，
    // 我们不能把看门狗的回调抛给同一个有可能会死锁的 Executor，
    // 而是直接打破 IoC，使用 std::thread 创建一个永远不会被 ROS 2 阻塞的纯净监控线程。
    // -------------------------------------------------------------------------
    std::atomic<bool> system_running{true};
    std::thread hardware_watchdog([&node, &system_running]() {
        while (system_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            auto current_time = node->now();
            auto last_heartbeat = node->get_last_heartbeat_time();
            
            // 如果刚启动还没心跳，或者心跳时间正常，跳过
            if (last_heartbeat.nanoseconds() == 0) continue;
            
            if ((current_time - last_heartbeat).seconds() > 1.0) {
                RCLCPP_FATAL(node->get_logger(), "🚨 [独立看门狗线程] 警告：发现执行器已被强行阻塞！心跳停滞超过 1 秒！急停！");
            }
        }
    });

    // -------------------------------------------------------------------------
    // 知识点：动态参数
    // 我们定义一个参数 "executor_type"，允许在不重新编译的情况下切换执行器
    // "thread_count" 则用于配置多线程执行器允许的最大线程数，0 表示不限制（取决于 CPU 核心数）
    // -------------------------------------------------------------------------
    node->declare_parameter("executor_type", "single");
    node->declare_parameter("thread_count", 0);
    std::string executor_type = node->get_parameter("executor_type").as_string();
    int thread_count = node->get_parameter("thread_count").as_int();

    RCLCPP_INFO(node->get_logger(), "配置信息: 执行器类型: [%s], 线程数: [%d] (0=不限)", 
                executor_type.c_str(), thread_count);

    // 根据参数值选择执行器逻辑
    if (executor_type == "multi")
    {
        // ---------------------------------------------------------------------
        // 知识点：MultiThreadedExecutor（多线程执行器）
        // 知识点：MultiThreadedExecutor（多线程执行器）
        // 可以通过构造函数参数传入线程数，以模拟计算平台算力受限的情况。
        // ---------------------------------------------------------------------
        rclcpp::ExecutorOptions options;
        // 如果这里传入的 thread_count == 1，那么多线程执行器实际上就退化成了单线程的行为逻辑（会产生互相阻塞）。
        rclcpp::executors::MultiThreadedExecutor executor(options, thread_count);
        
        RCLCPP_INFO(node->get_logger(), "正在使用多线程执行器 [并行模式]，分配线程数：%d", thread_count);
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
    system_running = false;
    if (hardware_watchdog.joinable()) {
        hardware_watchdog.join();
    }
    rclcpp::shutdown();
    return 0;
}
