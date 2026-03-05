#pragma once

/**
 * @file   image_processor.hpp
 * @brief  ImageProcessor 组件节点声明——流水线第二级，对图像做简单变换。
 *
 * 订阅 /pipeline/raw 并以 UniquePtr 回调签名接收消息（零拷贝接收）。
 * 对像素数据做按位取反变换后，以 unique_ptr 方式发布到 /pipeline/processed。
 * 通过日志打印消息指针地址来验证是否实现了零拷贝。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ros2_learning_composition
{

/**
 * @brief 订阅原始图像、反转像素后发布到下游的组件节点。
 *
 * 关键设计：
 * - 订阅回调使用 UniquePtr 签名，在 intra-process 模式下实现零拷贝接收
 * - 日志输出接收和发送的消息指针地址，便于观察零拷贝是否生效
 */
class ImageProcessor : public rclcpp::Node
{
public:
    /**
     * @brief 构造 ImageProcessor 并创建订阅器与发布器。
     * @param[in] options ROS 2 节点选项。
     */
    explicit ImageProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    /**
     * @brief 图像处理回调：反转像素数据并转发。
     * @param[in] msg 以 unique_ptr 接收的原始图像消息（所有权转移）。
     */
    void image_callback(sensor_msgs::msg::Image::UniquePtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  ///< 原始图像订阅器。
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;  ///< 处理后图像发布器。
    uint64_t processed_count_{0};  ///< 已处理帧计数。
};

}  // namespace ros2_learning_composition
