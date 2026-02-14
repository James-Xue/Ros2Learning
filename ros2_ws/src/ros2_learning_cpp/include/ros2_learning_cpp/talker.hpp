#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_learning_cpp
{
    // 改为继承 LifecycleNode
    class TalkerNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        // 类型别名，简化冗长的类型声明
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        using LifecyclePublisherPtr = rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr;

        // LifecycleNode 的构造函数需要 rclcpp::NodeOptions
        explicit TalkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        // 生命周期回调函数
        // 配置：on_configure (Unconfigured -> Inactive)
        CallbackReturn on_configure(const rclcpp_lifecycle::State &);

        // 激活：on_activate (Inactive -> Active)
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);

        // 停用：on_deactivate (Active -> Inactive)
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

        // 清理：on_cleanup (Inactive -> Unconfigured)
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

        // 关闭：on_shutdown (Any -> Finalized)
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

    private:
        void on_timer();

        // 注意：LifecyclePublisher 而不是普通 Publisher
        LifecyclePublisherPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_{0};
    };
}
