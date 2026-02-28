#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2_learning_parameters_advanced
{

/**
 * @brief 参数进阶演示节点
 *
 * 演示要点：
 *   1. declare_parameter — 带类型约束与描述信息
 *   2. add_on_set_parameters_callback — 运行时参数校验
 *   3. 从 YAML 文件加载初始值（通过 launch 文件注入）
 *   4. 定时读取参数并打印，验证动态修改效果
 */
class ParamDemoNode : public rclcpp::Node
{
public:
    explicit ParamDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    // 参数回调：校验新值是否合法，返回 SetParametersResult
    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter> & params);

    void on_timer();

    rclcpp::TimerBase::SharedPtr timer_;

    // 参数回调句柄，必须持有否则回调会被注销
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace ros2_learning_parameters_advanced
