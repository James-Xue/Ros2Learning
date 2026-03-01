#pragma once

/**
 * @file sensor_node.hpp
 * @brief SensorNode 生命周期节点声明。
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ros2_learning_lifecycle
{

/**
 * @class SensorNode
 * @brief 演示正确 Lifecycle 资源管理模式的传感器节点。
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
    /// 生命周期回调统一返回类型。
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /// Float64 生命周期发布器类型别名。
    using LifecyclePublisherFloat64 =
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>;

    /**
     * @brief 构造 SensorNode 并声明参数。
     * @param options ROS 2 节点选项。
     */
    explicit SensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 执行 Unconfigured -> Inactive，创建发布器。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Inactive -> Active，激活发布器并创建定时器。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Active -> Inactive，销毁定时器并停用发布器。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Inactive -> Unconfigured，释放发布器资源。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Any -> Finalized，释放所有资源。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief 提供给测试代码的发布器访问接口。
     * @return 内部生命周期发布器共享指针。
     */
    LifecyclePublisherFloat64::SharedPtr get_publisher() const { return publisher_; }

private:
    /**
     * @brief 定时器回调，发布模拟传感器数据。
     */
    void on_timer();

    /// 生命周期发布器（topic: /sensor_data）。
    LifecyclePublisherFloat64::SharedPtr publisher_;
    /// 仅在 Active 状态存在的 wall timer。
    rclcpp::TimerBase::SharedPtr timer_;
    /// 正弦波模拟数据相位累计值。
    double simulated_value_{0.0};
};

}  // namespace ros2_learning_lifecycle
