/**
 * @file   param_event_listener_node.cpp
 * @brief  参数事件监听节点实现：订阅 /parameter_events，打印全局参数变化
 */

#include "ros2_learning_parameters_advanced/param_event_listener_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace ros2_learning_parameters_advanced
{

// ─────────────────────────────────────────────────────────────────────────────
// 将 ParameterEvent 中的 ParameterValue 转为可读字符串
//
// ParameterEvent 消息使用底层 rcl_interfaces::msg::ParameterValue（含 type 字段），
// 而非 rclcpp::Parameter（后者提供 value_to_string()）。
// 因此需要手动按 type 字段分支序列化。
// ─────────────────────────────────────────────────────────────────────────────
static std::string param_value_to_str(const rcl_interfaces::msg::ParameterValue & v)
{
    // type 字段对应 rcl_interfaces::msg::ParameterType 中的常量
    switch (v.type) {
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
            return v.bool_value ? "true" : "false";
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
            return std::to_string(v.integer_value);
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
            return std::to_string(v.double_value);
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
            return "\"" + v.string_value + "\"";  // 加引号区分字符串与其他类型
        case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:    return "[byte_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:    return "[bool_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY: return "[integer_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:  return "[double_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:  return "[string_array]";
        default: return "<not_set>";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
ParamEventListenerNode::ParamEventListenerNode(const rclcpp::NodeOptions & options)
: Node("param_event_listener_node", options)
{
    // /parameter_events 是 ROS 2 中所有节点参数变化的全局广播话题。
    // 必须使用 rclcpp::ParameterEventsQoS() 而非默认 QoS：
    //   - profile：Reliable + Volatile + depth=1000
    //   - 与系统参数事件发布者的 QoS 完全匹配，保证在线期间事件可靠接收
    //   - 注意：Volatile 表示无历史缓存，晚于发布者启动仍会丢失历史事件
    sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        rclcpp::ParameterEventsQoS(),
        [this](const rcl_interfaces::msg::ParameterEvent & event) {
            on_parameter_event(event);
        });

    RCLCPP_INFO(get_logger(),
        "ParamEventListenerNode 已启动，监听 /parameter_events\n"
        "  尝试：ros2 param set /param_demo_node node_label hello");
}

// ─────────────────────────────────────────────────────────────────────────────
// 参数事件回调：打印其他节点的参数新增、修改、删除事件
// ─────────────────────────────────────────────────────────────────────────────
void ParamEventListenerNode::on_parameter_event(
    const rcl_interfaces::msg::ParameterEvent & event)
{
    // 过滤本节点自身产生的事件，避免 declare_parameter 时的噪声干扰输出
    if (event.node == get_fully_qualified_name()) {
        return;
    }

    // 新增参数：节点启动时每次 declare_parameter 都会触发一条 new_parameters 事件
    for (const auto & p : event.new_parameters) {
        RCLCPP_INFO(get_logger(),
            "[event:new]     节点=%-40s  参数=%-25s  值=%s",
            event.node.c_str(), p.name.c_str(), param_value_to_str(p.value).c_str());
    }

    // 修改参数：ros2 param set 或 set_parameter() 触发
    for (const auto & p : event.changed_parameters) {
        RCLCPP_INFO(get_logger(),
            "[event:changed] 节点=%-40s  参数=%-25s  新值=%s",
            event.node.c_str(), p.name.c_str(), param_value_to_str(p.value).c_str());
    }

    // 删除参数：undeclare_parameter() 触发（普通参数较少见）
    for (const auto & p : event.deleted_parameters) {
        RCLCPP_INFO(get_logger(),
            "[event:deleted] 节点=%-40s  参数=%-25s",
            event.node.c_str(), p.name.c_str());
    }
}

}  // namespace ros2_learning_parameters_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_parameters_advanced::ParamEventListenerNode)
