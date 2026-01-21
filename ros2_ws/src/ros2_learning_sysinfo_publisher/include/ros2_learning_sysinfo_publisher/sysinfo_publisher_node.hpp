// sysinfo_publisher_node.hpp
// 系统信息发布器节点类

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros2_learning_sysinfo_publisher
{

/**
 * @brief 系统信息发布器节点
 * 
 * 功能：
 * - 定期读取系统信息（CPU、内存、网络等）
 * - 构建 JSON 格式的消息
 * - 发布到指定话题
 * 
 * 参数：
 * - topic: 发布话题名称（默认: /ros2_learning/sysinfo）
 * - publish_rate_hz: 发布频率（默认: 1.0 Hz）
 */
class SysInfoPublisherNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    SysInfoPublisherNode();

    /**
     * @brief 析构函数
     */
    ~SysInfoPublisherNode() override = default;

private:
    /**
     * @brief 定时器回调函数
     * 
     * 读取系统信息并发布 JSON 消息
     */
    void onTimer();

    std::string m_topic;           ///< 发布话题名称
    double m_publishRateHz;        ///< 发布频率 (Hz)
    
    /// ROS 2 发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
    
    /// 定时器
    rclcpp::TimerBase::SharedPtr m_timer;
};

} // namespace ros2_learning_sysinfo_publisher
