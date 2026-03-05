/**
 * @file   image_processor.cpp
 * @brief  ImageProcessor 组件节点实现——接收原始图像、反转像素后转发。
 *
 * 使用 UniquePtr 回调签名实现 intra-process 零拷贝接收。
 * 日志打印消息指针地址，便于观察零拷贝是否生效。
 */

#include "ros2_learning_composition/image_processor.hpp"

#include <memory>

#include <rclcpp_components/register_node_macro.hpp>

namespace ros2_learning_composition
{

ImageProcessor::ImageProcessor(const rclcpp::NodeOptions & options)
: Node("image_processor", options)
{
    // 发布器：处理后的图像（相对话题名，支持 namespace/remap）
    publisher_ = create_publisher<sensor_msgs::msg::Image>("pipeline/processed", 10);

    // 订阅器：使用 UniquePtr 回调签名，在 intra-process 模式下实现零拷贝接收
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "pipeline/raw", 10,
        std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
        "ImageProcessor 已启动: intra_process=%s",
        options.use_intra_process_comms() ? "ON" : "OFF");
}

void ImageProcessor::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    RCLCPP_DEBUG(get_logger(),
        "收到帧, 输入 data ptr=%p",
        static_cast<const void *>(msg->data.data()));

    // 简单变换：按位取反（反转像素值）
    for (auto & pixel : msg->data) {
        pixel = static_cast<uint8_t>(~pixel);
    }

    ++processed_count_;

    RCLCPP_DEBUG(get_logger(),
        "转发帧 #%lu, 输出 data ptr=%p",
        processed_count_, static_cast<const void *>(msg->data.data()));

    // 以 unique_ptr 方式发布到下游，继续保持零拷贝链
    publisher_->publish(std::move(msg));
}

}  // namespace ros2_learning_composition

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_composition::ImageProcessor)
