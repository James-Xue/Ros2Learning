/**
 * @file   image_producer.cpp
 * @brief  ImageProducer 组件节点实现——定时生成模拟图像并以零拷贝方式发布。
 */

#include "ros2_learning_composition/image_producer.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace ros2_learning_composition
{

ImageProducer::ImageProducer(const rclcpp::NodeOptions & options)
: Node("image_producer", options)
{
    // 声明参数（含范围约束）
    auto width_desc = rcl_interfaces::msg::ParameterDescriptor{};
    width_desc.description = "图像宽度（像素），合法范围 [1, 4096]";
    width_desc.integer_range.resize(1);
    width_desc.integer_range[0].from_value = 1;
    width_desc.integer_range[0].to_value = 4096;
    width_desc.integer_range[0].step = 1;
    declare_parameter("image_width", 64, width_desc);

    auto height_desc = rcl_interfaces::msg::ParameterDescriptor{};
    height_desc.description = "图像高度（像素），合法范围 [1, 4096]";
    height_desc.integer_range.resize(1);
    height_desc.integer_range[0].from_value = 1;
    height_desc.integer_range[0].to_value = 4096;
    height_desc.integer_range[0].step = 1;
    declare_parameter("image_height", 64, height_desc);

    auto rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
    rate_desc.description = "发布频率（Hz），合法范围 (0, 1000]";
    rate_desc.floating_point_range.resize(1);
    rate_desc.floating_point_range[0].from_value = 0.001;
    rate_desc.floating_point_range[0].to_value = 1000.0;
    rate_desc.floating_point_range[0].step = 0.0;
    declare_parameter("publish_rate_hz", 10.0, rate_desc);

    image_width_ = get_parameter("image_width").as_int();
    image_height_ = get_parameter("image_height").as_int();
    double rate_hz = get_parameter("publish_rate_hz").as_double();

    // 二次防御校验（descriptor 不一定能拦截所有场景）
    if (image_width_ <= 0 || image_width_ > 4096 ||
        image_height_ <= 0 || image_height_ > 4096)
    {
        throw std::invalid_argument(
            "image_width/height 必须在 [1, 4096] 范围内，当前: " +
            std::to_string(image_width_) + "x" + std::to_string(image_height_));
    }
    if (rate_hz <= 0.0 || rate_hz > 1000.0) {
        throw std::invalid_argument(
            "publish_rate_hz 必须在 (0, 1000] 范围内，当前: " + std::to_string(rate_hz));
    }

    // 创建发布器（使用相对话题名，支持 namespace/remap）
    publisher_ = create_publisher<sensor_msgs::msg::Image>("pipeline/raw", 10);

    // 创建定时器
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ImageProducer::on_timer, this));

    RCLCPP_INFO(get_logger(),
        "ImageProducer 已启动: %dx%d @ %.1f Hz, intra_process=%s",
        image_width_, image_height_, rate_hz,
        options.use_intra_process_comms() ? "ON" : "OFF");
}

void ImageProducer::on_timer()
{
    auto msg = create_synthetic_image();

    RCLCPP_DEBUG(get_logger(),
        "发布帧 #%u, data ptr=%p, size=%zu",
        frame_count_, static_cast<const void *>(msg->data.data()), msg->data.size());

    // 关键：使用 std::move 发布 unique_ptr，启用零拷贝
    publisher_->publish(std::move(msg));
    ++frame_count_;
}

sensor_msgs::msg::Image::UniquePtr ImageProducer::create_synthetic_image()
{
    auto msg = std::make_unique<sensor_msgs::msg::Image>();

    // 填充 header（时间戳用于下游延迟计算）
    msg->header.stamp = now();
    msg->header.frame_id = "camera_frame";

    // 图像参数
    msg->width = static_cast<uint32_t>(image_width_);
    msg->height = static_cast<uint32_t>(image_height_);
    msg->encoding = "rgb8";
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(image_width_ * 3);  // 3 bytes per pixel (RGB)

    // 合成像素数据：每帧用不同值填充
    size_t data_size = static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height);
    msg->data.resize(data_size);
    uint8_t fill_value = static_cast<uint8_t>(frame_count_ % 256);
    std::fill(msg->data.begin(), msg->data.end(), fill_value);

    return msg;
}

}  // namespace ros2_learning_composition

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_composition::ImageProducer)
