#pragma once

#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ros2_learning_lifecycle
{

/**
 * ProcessorNode — 演示级联 LifecycleNode：订阅 SensorNode 数据并发布处理结果。
 *
 * 核心演示点：
 * 1. 订阅回调中主动检查自身状态，避免在 Inactive 时处理数据
 * 2. on_configure 错误注入：通过参数 configure_behavior 返回 SUCCESS/FAILURE/ERROR
 * 3. on_error 回调：从 ErrorProcessing 状态恢复到 Unconfigured
 *
 * 参数：
 *   configure_behavior (string, default="success")
 *     - "success" → 正常 configure（Unconfigured → Inactive）
 *     - "failure" → configure 失败，回到 Unconfigured（不进入 ErrorProcessing）
 *     - "error"   → configure 出错，进入 ErrorProcessing，触发 on_error
 *   window_size (int, [1,20], default=5) — 移动平均窗口大小
 *
 * 订阅：/sensor_data   (std_msgs/Float64)
 * 发布：/processed_data (std_msgs/Float64) — 窗口移动平均值
 */
class ProcessorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using LifecyclePublisherFloat64 =
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>;

    explicit ProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

    // 测试可访问接口
    LifecyclePublisherFloat64::SharedPtr get_publisher() const { return publisher_; }

private:
    void on_sensor_data(const std_msgs::msg::Float64 & msg);
    void cleanup_resources();

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    LifecyclePublisherFloat64::SharedPtr publisher_;
    std::deque<double> window_;
};

}  // namespace ros2_learning_lifecycle
