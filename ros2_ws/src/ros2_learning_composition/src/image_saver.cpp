/**
 * @file   image_saver.cpp
 * @brief  ImageSaver 生命周期组件节点实现——统计帧数与延迟。
 *
 * 继承 rclcpp_lifecycle::LifecycleNode，演示生命周期节点作为 rclcpp 组件注册。
 * 状态转换控制数据采集启停，on_deactivate 时输出统计摘要。
 */

#include "ros2_learning_composition/image_saver.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace ros2_learning_composition
{

ImageSaver::ImageSaver(const rclcpp::NodeOptions & options)
: LifecycleNode("image_saver", options)
{
    RCLCPP_INFO(get_logger(), "ImageSaver 已创建（Unconfigured 状态）");
}

ImageSaver::CallbackReturn
ImageSaver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_configure: 创建订阅器");

    // 创建订阅器（UniquePtr 回调签名，相对话题名）
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "pipeline/processed", 10,
        std::bind(&ImageSaver::image_callback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
}

ImageSaver::CallbackReturn
ImageSaver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_activate: 重置统计并开始采集");

    // 重置统计（加锁保护）
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        frame_count_ = 0;
        total_latency_sec_ = 0.0;
    }
    active_.store(true);

    return CallbackReturn::SUCCESS;
}

ImageSaver::CallbackReturn
ImageSaver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    active_.store(false);

    // 打印统计摘要（加锁保护）
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (frame_count_ > 0) {
        double avg_latency_ms = (total_latency_sec_ / static_cast<double>(frame_count_)) * 1000.0;
        RCLCPP_INFO(get_logger(),
            "on_deactivate: 统计摘要 — 帧数=%lu, 平均延迟=%.3f ms",
            frame_count_, avg_latency_ms);
    } else {
        RCLCPP_INFO(get_logger(), "on_deactivate: 未接收到任何帧");
    }

    return CallbackReturn::SUCCESS;
}

ImageSaver::CallbackReturn
ImageSaver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_cleanup: 释放订阅器");
    subscription_.reset();
    return CallbackReturn::SUCCESS;
}

ImageSaver::CallbackReturn
ImageSaver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "on_shutdown: 释放所有资源");
    subscription_.reset();
    active_.store(false);
    return CallbackReturn::SUCCESS;
}

void ImageSaver::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    if (!active_.load()) {
        return;
    }

    // 计算延迟：当前时间 - 消息时间戳
    auto now_time = now();
    auto stamp = rclcpp::Time(msg->header.stamp);
    double latency_sec = (now_time - stamp).seconds();

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++frame_count_;
        total_latency_sec_ += latency_sec;
    }

    RCLCPP_DEBUG(get_logger(),
        "收到帧, 延迟=%.3f ms, data ptr=%p",
        latency_sec * 1000.0,
        static_cast<const void *>(msg->data.data()));
}

}  // namespace ros2_learning_composition

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_composition::ImageSaver)
