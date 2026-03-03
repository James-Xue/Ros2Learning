/**
 * @file   param_demo_node.hpp
 * @brief  参数进阶演示节点：带描述符声明、运行时校验回调与定时读取
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2_learning_parameters_advanced
{

/**
 * @brief 参数进阶演示节点
 *
 * 演示要点：
 *   1. declare_parameter — 带类型约束与描述信息（IntegerRange / FloatingPointRange）
 *   2. add_on_set_parameters_callback — 运行时参数合法性校验（fail-fast 语义）
 *   3. 从 YAML 文件加载初始值（通过 launch 文件注入），并在构造函数中手动校验
 *   4. 定时读取参数快照，验证 `ros2 param set` 的动态修改效果
 */
class ParamDemoNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数：按阶段完成参数声明、初值校验、回调注册、定时器创建
     *
     * @param[in] options  节点选项（NodeOptions），支持 Component 模式组合使用
     * @throws std::invalid_argument  若 YAML/命令行注入的初值非法：
     *         `scale_factor <= 0.0` 或 `node_label` 为空字符串
     */
    explicit ParamDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    /**
     * @brief 参数修改前的合法性校验回调（由 add_on_set_parameters_callback 注册）
     *
     * 遇到第一个非法参数立即拒绝整批修改（fail-fast），
     * 防止部分修改生效导致状态不一致。
     *
     * @param[in] params  本次待修改的参数列表
     * @return SetParametersResult  successful=true 表示全部通过，false 表示拒绝
     */
    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter> & params);

    /**
     * @brief 定时器回调：每秒打印当前参数快照，验证动态修改是否生效
     */
    void on_timer();

    rclcpp::TimerBase::SharedPtr timer_;  ///< 固定 1 s 周期的墙钟定时器

    /// 参数修改前回调的句柄；必须持有，否则 SharedPtr 析构后回调立即被注销
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace ros2_learning_parameters_advanced
