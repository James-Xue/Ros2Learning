/**
 * @file   param_event_listener_node.hpp
 * @brief  参数事件监听节点：订阅全局 /parameter_events，打印任意节点的参数变化
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

namespace ros2_learning_parameters_advanced
{

/**
 * @brief 参数事件监听节点
 *
 * 订阅全局 /parameter_events 话题，打印**除自身外**所有节点的参数
 * 新增（new_parameters）、修改（changed_parameters）、
 * 删除（deleted_parameters）事件。自身事件被过滤以减少噪声。
 *
 * 演示要点：
 *   - rcl_interfaces/msg/ParameterEvent 消息结构（底层类型，无 rclcpp 封装）
 *   - 使用 rclcpp::ParameterEventsQoS() 与系统参数发布者 QoS 匹配（Reliable + Volatile）
 *   - 通过 node 字段过滤自身事件，专注观察其他节点的参数变化
 *   - 与 add_on_set_parameters_callback 的互补关系：
 *       前者用于拦截本节点参数修改，后者用于旁观全局参数事件
 */
class ParamEventListenerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数：创建 /parameter_events 订阅
     *
     * @param[in] options  节点选项，支持 Component 模式
     */
    explicit ParamEventListenerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    /**
     * @brief 参数事件回调：打印新增、修改、删除三类事件
     *
     * 自身节点产生的事件会被过滤，仅输出其他节点的参数变化。
     *
     * @param[in] event  /parameter_events 话题收到的事件消息
     */
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event);

    /// /parameter_events 订阅者（使用 ParameterEventsQoS）
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_;
};

}  // namespace ros2_learning_parameters_advanced
