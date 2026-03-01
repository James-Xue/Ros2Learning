#pragma once

/**
 * @file processor_node.hpp
 * @brief ProcessorNode 生命周期节点声明。
 */

#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ros2_learning_lifecycle
{

/**
 * @class ProcessorNode
 * @brief 订阅传感器数据并发布处理结果的 LifecycleNode。
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
    /// 生命周期回调统一返回类型。
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /// Float64 生命周期发布器类型别名。
    using LifecyclePublisherFloat64 =
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>;

    /**
     * @brief 构造 ProcessorNode 并声明参数。
     * @param options ROS 2 节点选项。
     */
    explicit ProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 执行 Unconfigured -> Inactive（或 FAILURE/ERROR 分支）。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Inactive -> Active，激活发布器。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Active -> Inactive，停用发布器。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Inactive -> Unconfigured，清理资源。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief 执行 Any -> Finalized，清理资源。
     * @param state 当前生命周期状态（由框架传入）。
     * @return 生命周期回调结果。
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief ErrorProcessing 状态回调，执行恢复逻辑。
     * @param state 触发错误前的状态。
     * @return 返回 SUCCESS 将回到 Unconfigured。
     */
    CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief 提供给测试代码的发布器访问接口。
     * @return 内部生命周期发布器共享指针。
     */
    LifecyclePublisherFloat64::SharedPtr get_publisher() const { return publisher_; }

private:
    /**
     * @brief 订阅 /sensor_data 的回调，计算并发布窗口均值。
     * @param msg 输入传感器消息。
     */
    void on_sensor_data(const std_msgs::msg::Float64 & msg);

    /**
     * @brief 清理订阅、发布器和内部缓存窗口。
     */
    void cleanup_resources();

    /// 传感器数据订阅器（topic: /sensor_data）。
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    /// 处理结果生命周期发布器（topic: /processed_data）。
    LifecyclePublisherFloat64::SharedPtr publisher_;
    /// 移动平均窗口缓存。
    std::deque<double> window_;
};

}  // namespace ros2_learning_lifecycle
