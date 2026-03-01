#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ros2_learning_lifecycle
{

/**
 * SensorNode — 演示正确的 LifecycleNode timer/publisher 管理规范。
 *
 * 状态转换与资源生命周期对应关系：
 *   on_configure  : 创建 LifecyclePublisher（此时 publisher 未激活，不发数据）
 *   on_activate   : 激活 publisher（调用父类），创建 timer（开始发数据）
 *   on_deactivate : 销毁 timer（停止发数据），停用 publisher（调用父类）
 *   on_cleanup    : 销毁 publisher（回到 Unconfigured）
 *   on_shutdown   : 销毁 timer + publisher（终态）
 *
 * 关键修正：timer 应在 on_activate 创建、on_deactivate 销毁，
 * 而非在 on_configure 创建后依赖 is_activated() 做 guard（anti-pattern）。
 *
 * 发布话题：/sensor_data (std_msgs/Float64)
 * 参数：
 *   sensor_id      (string, default="sensor_0") — 传感器编号，用于日志标识
 *   publish_rate_hz (int, [1,10], default=2)     — 发布频率
 */
class SensorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using LifecyclePublisherFloat64 =
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>;

    explicit SensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    // 测试可访问接口
    LifecyclePublisherFloat64::SharedPtr get_publisher() const { return publisher_; }

private:
    void on_timer();

    LifecyclePublisherFloat64::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double simulated_value_{0.0};
};

}  // namespace ros2_learning_lifecycle
