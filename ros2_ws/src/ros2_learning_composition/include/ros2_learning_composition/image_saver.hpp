#pragma once

/**
 * @file   image_saver.hpp
 * @brief  ImageSaver 生命周期组件节点声明——流水线第三级，统计接收帧数与延迟。
 *
 * 继承 rclcpp_lifecycle::LifecycleNode，演示 LifecycleNode 作为 rclcpp 组件注册。
 * 生命周期状态控制数据采集的启停：
 *   on_configure  : 创建订阅器
 *   on_activate   : 重置统计，开始计数
 *   on_deactivate : 打印统计摘要
 *   on_cleanup    : 释放订阅器
 */

#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ros2_learning_composition
{

/**
 * @brief 生命周期组件节点，订阅处理后的图像并记录帧率与延迟统计。
 *
 * 设计要点：
 * - 仅在 Active 状态下累积统计数据
 * - on_deactivate 时输出帧数与平均延迟摘要
 * - 作为 rclcpp_components 注册，可被 ComposableNodeContainer 加载
 *
 * @note 线程安全：active_ 使用 std::atomic<bool>，统计字段（frame_count_、
 *       total_latency_sec_）通过 stats_mutex_ 保护，确保回调线程与外部
 *       getter / 生命周期回调之间无数据竞争。
 */
class ImageSaver : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// 生命周期回调统一返回类型。
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief 构造 ImageSaver 生命周期组件节点。
     * @param[in] options ROS 2 节点选项。
     */
    explicit ImageSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief on_configure：创建订阅器。
     * @param[in] state 当前生命周期状态。
     * @return SUCCESS 表示转换成功。
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief on_activate：重置统计计数器，开始数据采集。
     * @param[in] state 当前生命周期状态。
     * @return SUCCESS 表示转换成功。
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief on_deactivate：停止采集并打印统计摘要。
     * @param[in] state 当前生命周期状态。
     * @return SUCCESS 表示转换成功。
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief on_cleanup：释放订阅器资源。
     * @param[in] state 当前生命周期状态。
     * @return SUCCESS 表示转换成功。
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief on_shutdown：释放所有资源（终态）。
     * @param[in] state 当前生命周期状态。
     * @return SUCCESS 表示转换成功。
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief 获取已接收的帧数（线程安全，供测试使用）。
     * @return 已接收帧数。
     */
    uint64_t get_frame_count() const
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return frame_count_;
    }

    /**
     * @brief 获取累计延迟总和（线程安全，供测试使用）。
     * @return 累计延迟（秒）。
     */
    double get_total_latency() const
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return total_latency_sec_;
    }

private:
    /**
     * @brief 图像接收回调：累积帧数和延迟统计。
     * @param[in] msg 接收到的处理后图像消息（所有权转移）。
     */
    void image_callback(sensor_msgs::msg::Image::UniquePtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  ///< 处理后图像订阅器。
    mutable std::mutex stats_mutex_;  ///< 保护统计字段的互斥锁。
    uint64_t frame_count_{0};         ///< 已接收帧数（受 stats_mutex_ 保护）。
    double total_latency_sec_{0.0};   ///< 累计延迟（秒，受 stats_mutex_ 保护）。
    std::atomic<bool> active_{false}; ///< 是否处于 Active 状态（原子，控制统计采集）。
};

}  // namespace ros2_learning_composition
