#pragma once

#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_multithreading
{
    /**
     * @brief 阻塞演示节点
     * 
     * 这个节点用于演示单线程执行器 (SingleThreadedExecutor) 和多线程执行器 (MultiThreadedExecutor) 
     * 在处理耗时回调时的不同表现。
     * 
     * 核心知识点：
     * 1. CallbackGroup (回调组)：用于控制回调函数的并发规则。
     * 2. Reentrant (可重入) vs MutuallyExclusive (互斥) 组的区别。
     */
    class BlockingNode : public rclcpp::Node
    {
    public:
        BlockingNode();

    private:
        /**
         * @brief 心跳回调函数 (模拟高频、实时性要求高的任务)
         * 
         * 例如：发送电机控制指令、发布关节状态、喂看门狗。
         * 如果这个回调被阻塞，机器人可能会失控或急停。
         */
        void on_heartbeat();

        /**
         * @brief 繁重计算回调函数 (模拟耗时、阻塞的任务)
         * 
         * 例如：全局路径规划、图像处理、点云配准。
         * 这个函数会占用 CPU 较长时间。
         */
        void on_heavy_calculation();

        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::TimerBase::SharedPtr heavy_timer_;

        // 回调组指针
        // 在 ROS 2 中，如果不指定 CallbackGroup，所有回调默认属于一个 "Default MutuallyExclusive Group"。
        // 也就是说，默认情况下，节点内的所有回调都是互斥的（排队执行）。
        rclcpp::CallbackGroup::SharedPtr callback_group_1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_2_;
    };
}
