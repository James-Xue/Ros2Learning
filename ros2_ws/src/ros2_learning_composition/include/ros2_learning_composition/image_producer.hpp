#pragma once

/**
 * @file   image_producer.hpp
 * @brief  ImageProducer 组件节点声明——流水线第一级，生成模拟图像数据。
 *
 * 使用 unique_ptr 发布方式启用 intra-process 零拷贝通信。
 * 当所有订阅者与发布者位于同一 executor 且启用 use_intra_process_comms 时，
 * 消息将以指针移动而非序列化拷贝方式传递。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ros2_learning_composition
{

/**
 * @brief 定时生成模拟图像并发布到 pipeline/raw 的组件节点。
 *
 * 图像为合成 RGB8 数据，每帧递增填充值以区分不同帧。
 * 关键设计：使用 publisher->publish(std::move(unique_ptr)) 实现零拷贝发布。
 *
 * 参数（均在构造时校验，非法值将抛出 std::invalid_argument）：
 *   image_width      (int, default=64, 合法范围 [1, 4096])   — 图像宽度
 *   image_height     (int, default=64, 合法范围 [1, 4096])   — 图像高度
 *   publish_rate_hz  (double, default=10.0, 合法范围 (0, 1000]) — 发布频率
 *
 * @note 线程安全：所有成员仅在定时器回调中访问，由 executor 序列化调用，无需额外同步。
 */
class ImageProducer : public rclcpp::Node
{
public:
    /**
     * @brief 构造 ImageProducer 并创建发布器与定时器。
     * @param[in] options ROS 2 节点选项（含 use_intra_process_comms 等）。
     * @throws std::invalid_argument 当参数值非法时（width/height <= 0 或 > 4096，rate_hz <= 0）。
     */
    explicit ImageProducer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    /**
     * @brief 定时器回调：构造模拟图像消息并以 unique_ptr 方式发布。
     */
    void on_timer();

    /**
     * @brief 构造一帧模拟图像数据。
     * @return 堆分配的 Image 消息（unique_ptr）。
     */
    sensor_msgs::msg::Image::UniquePtr create_synthetic_image();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;  ///< 原始图像发布器（topic: pipeline/raw）。
    rclcpp::TimerBase::SharedPtr timer_;  ///< 定时发布触发器。
    uint32_t frame_count_{0};  ///< 帧计数器，用于生成不同像素值。
    int image_width_{64};   ///< 图像宽度（像素），合法范围 [1, 4096]。
    int image_height_{64};  ///< 图像高度（像素），合法范围 [1, 4096]。
};

}  // namespace ros2_learning_composition
