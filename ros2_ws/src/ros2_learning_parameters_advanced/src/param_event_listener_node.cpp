#include "ros2_learning_parameters_advanced/param_event_listener_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace ros2_learning_parameters_advanced
{

// 将 rcl_interfaces::msg::ParameterValue 转为可读字符串
static std::string param_value_to_str(const rcl_interfaces::msg::ParameterValue & v)
{
    switch (v.type) {
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
            return v.bool_value ? "true" : "false";
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
            return std::to_string(v.integer_value);
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
            return std::to_string(v.double_value);
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
            return "\"" + v.string_value + "\"";
        case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
            return "[byte_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
            return "[bool_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
            return "[integer_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return "[double_array]";
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
            return "[string_array]";
        default:
            return "<not_set>";
    }
}

ParamEventListenerNode::ParamEventListenerNode(const rclcpp::NodeOptions & options)
: Node("param_event_listener_node", options)
{
    // /parameter_events 是 ROS 2 中所有节点参数变化的全局广播话题
    // 必须使用 rclcpp::ParameterEventsQoS() 而非普通 QoS(10)：
    //   - 默认 profile 为 Reliable + Volatile + depth=1000
    //   - 与系统参数事件发布者 QoS 匹配，避免晚启动时丢失事件
    sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        rclcpp::ParameterEventsQoS(),
        [this](const rcl_interfaces::msg::ParameterEvent & event) {
            this->on_parameter_event(event);
        });

    RCLCPP_INFO(get_logger(),
        "ParamEventListenerNode 已启动，监听 /parameter_events\n"
        "  尝试：ros2 param set /param_demo_node node_label hello");
}

void ParamEventListenerNode::on_parameter_event(
    const rcl_interfaces::msg::ParameterEvent & event)
{
    // 过滤掉本节点自身产生的事件（避免干扰输出）
    if (event.node == this->get_fully_qualified_name()) {
        return;
    }

    // 新增参数（节点启动时 declare_parameter 会触发）
    for (const auto & p : event.new_parameters) {
        RCLCPP_INFO(get_logger(),
            "[event:new]     节点=%-40s  参数=%-25s  值=%s",
            event.node.c_str(), p.name.c_str(),
            param_value_to_str(p.value).c_str());
    }

    // 修改参数（ros2 param set 触发）
    for (const auto & p : event.changed_parameters) {
        RCLCPP_INFO(get_logger(),
            "[event:changed] 节点=%-40s  参数=%-25s  新值=%s",
            event.node.c_str(), p.name.c_str(),
            param_value_to_str(p.value).c_str());
    }

    // 删除参数（undeclare_parameter 触发）
    for (const auto & p : event.deleted_parameters) {
        RCLCPP_INFO(get_logger(),
            "[event:deleted] 节点=%-40s  参数=%-25s",
            event.node.c_str(), p.name.c_str());
    }
}

}  // namespace ros2_learning_parameters_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_parameters_advanced::ParamEventListenerNode)
