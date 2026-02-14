#include "ros2_learning_multithreading/blocking_node.hpp"
#include <thread>
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace ros2_learning_multithreading
{
    BlockingNode::BlockingNode()
        : Node("blocking_node")
    {
        // ========================== 知识点：回调组 ==========================
        // ROS 2 的调度是两层结构：Executor(执行器) -> CallbackGroup(回调组)。
        // 
        // 1. MutuallyExclusive (互斥组): 
        //    这个组里的所有回调，同一时刻只能有一个在运行。就像上厕所要排队。
        //    默认情况下，Node 里的所有回调都在默认的互斥组里。
        //
        // 2. Reentrant (可重入组):
        //    这个组里的回调可以并行运行（前提是 Executor 有多个线程）。
        //
        // 为了让两个定时器能并行（在 MultiThreadedExecutor 下），
        // 我们必须把它们分到 **不同** 的互斥组，或者同一个可重入组。
        // 这里我们选择把它们分到两个独立的互斥组。
        
        callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // ========================== 定时器配置 ==========================
        
        // 1. 心跳定时器 (500ms 一次) -> 分配给 Group 1
        // 模拟：底盘控制指令发送
        heartbeat_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&BlockingNode::on_heartbeat, this),
            callback_group_1_);

        // 2. 繁重计算定时器 (3秒 一次) -> 分配给 Group 2
        // 模拟：全局路径规划
        heavy_timer_ = this->create_wall_timer(
            3000ms,
            std::bind(&BlockingNode::on_heavy_calculation, this),
            callback_group_2_);
            
        RCLCPP_INFO(get_logger(), "阻塞演示节点已启动。观察线程行为...");
        RCLCPP_INFO(get_logger(), "如果配置为单线程执行器，心跳将被计算阻塞。");
        RCLCPP_INFO(get_logger(), "如果配置为多线程执行器，心跳将独立跳动。");
    }

    void BlockingNode::on_heartbeat()
    {
        // 打印当前线程 ID，用于验证是否在不同线程运行
        std::stringstream ss;
        ss << std::this_thread::get_id();
        
        // 使用 INFO 级别打印心跳
        RCLCPP_INFO(get_logger(), "[心跳] 咚! 线程 ID: %s", ss.str().c_str());
    }

    void BlockingNode::on_heavy_calculation()
    {
        std::stringstream ss;
        ss << std::this_thread::get_id();
        
        // 模拟一个耗时 2 秒的操作
        RCLCPP_WARN(get_logger(), "[计算] 开始复杂计算... 线程 ID: %s (预计阻塞 2秒)", ss.str().c_str());
        
        // 强行阻塞线程 2 秒
        std::this_thread::sleep_for(2000ms);
        
        RCLCPP_WARN(get_logger(), "[计算] 计算完成！");
    }
}
