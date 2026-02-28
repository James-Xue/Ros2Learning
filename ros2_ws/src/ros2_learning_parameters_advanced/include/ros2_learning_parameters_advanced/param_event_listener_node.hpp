#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

namespace ros2_learning_parameters_advanced
{

/**
 * @brief 参数事件监听节点
 *
 * 订阅全局 /parameter_events 话题，打印所有节点的参数
 * 新增（new_parameters）、修改（changed_parameters）、
 * 删除（deleted_parameters）事件。
 *
 * 演示要点：
 *   - rcl_interfaces/msg/ParameterEvent 结构
 *   - 通过 node 字段过滤感兴趣的节点
 *   - 参数事件与 add_on_set_parameters_callback 的互补关系
 */
class ParamEventListenerNode : public rclcpp::Node
{
public:
    explicit ParamEventListenerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event);

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_;
};

}  // namespace ros2_learning_parameters_advanced
