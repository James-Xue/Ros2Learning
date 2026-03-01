#include "ros2_learning_lifecycle/processor_node.hpp"

/**
 * @file processor_node.cpp
 * @brief ProcessorNode 生命周期回调与数据处理实现。
 */

#include <numeric>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_learning_lifecycle
{

/**
 * @brief 构造节点并声明行为注入与窗口参数。
 * @param options ROS 2 节点选项。
 */
ProcessorNode::ProcessorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("processor_node", options)
{
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description =
            "configure 阶段的行为注入：\n"
            "  'success' — 正常完成（默认）\n"
            "  'failure' — 返回 FAILURE，节点回到 Unconfigured\n"
            "  'error'   — 返回 ERROR，节点进入 ErrorProcessing，触发 on_error";
        this->declare_parameter("configure_behavior", "success", desc);
    }
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "移动平均窗口大小，范围 [1, 20]";
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = 1;
        range.to_value   = 20;
        range.step       = 1;
        desc.integer_range.push_back(range);
        this->declare_parameter("window_size", 5, desc);
    }

    RCLCPP_INFO(get_logger(), "[ProcessorNode] Unconfigured — 节点已构造，等待 configure 指令");
}

/**
 * @brief 生命周期回调：Unconfigured -> Inactive（或失败/错误分支）。
 * @param state 当前生命周期状态（未使用）。
 * @return SUCCESS/FAILURE/ERROR，取决于 configure_behavior 参数。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    const std::string behavior = this->get_parameter("configure_behavior").as_string();

    if (behavior == "failure") {
        RCLCPP_WARN(get_logger(),
            "[ProcessorNode] on_configure → FAILURE（注入）"
            "  节点将回到 Unconfigured，不进入 ErrorProcessing");
        return CallbackReturn::FAILURE;
    }

    if (behavior == "error") {
        RCLCPP_ERROR(get_logger(),
            "[ProcessorNode] on_configure → ERROR（注入）"
            "  节点将进入 ErrorProcessing，触发 on_error 回调");
        return CallbackReturn::ERROR;
    }

    // behavior == "success"（默认）
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "sensor_data", 10,
        std::bind(&ProcessorNode::on_sensor_data, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_data", 10);
    window_.clear();

    RCLCPP_INFO(get_logger(),
        "[ProcessorNode] on_configure ✓  subscription + publisher 已创建");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期回调：Inactive -> Active。
 * @param state 当前生命周期状态。
 * @return SUCCESS 表示发布器已激活。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_activate(const rclcpp_lifecycle::State & state)
{
    LifecycleNode::on_activate(state);
    RCLCPP_INFO(get_logger(), "[ProcessorNode] on_activate ✓  publisher 已激活，开始处理数据");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期回调：Active -> Inactive。
 * @param state 当前生命周期状态。
 * @return SUCCESS 表示发布器已停用。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "[ProcessorNode] on_deactivate ✓  publisher 已停用，停止发布结果");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期回调：Inactive -> Unconfigured。
 * @param state 当前生命周期状态（未使用）。
 * @return SUCCESS 表示资源已清理。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    cleanup_resources();
    RCLCPP_INFO(get_logger(), "[ProcessorNode] on_cleanup ✓  资源已释放，回到 Unconfigured");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期回调：Any -> Finalized。
 * @param state 触发 shutdown 前的状态。
 * @return SUCCESS 表示终态清理完成。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(),
        "[ProcessorNode] on_shutdown  从状态 '%s' 关闭", state.label().c_str());
    cleanup_resources();
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期错误回调：ErrorProcessing -> Unconfigured（返回 SUCCESS）。
 * @param state 触发错误前的状态。
 * @return SUCCESS 表示错误已处理并允许恢复。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_ERROR(get_logger(),
        "[ProcessorNode] on_error  触发！前一状态='%s'  正在清理并尝试恢复...",
        state.label().c_str());

    // 确保部分初始化的资源被清理
    cleanup_resources();

    RCLCPP_WARN(get_logger(),
        "[ProcessorNode] on_error  恢复成功 → 节点将回到 Unconfigured。"
        "  可以重新 configure（修改参数后）");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 订阅回调，计算窗口均值并在 Active 状态下发布。
 * @param msg 传感器输入数据。
 */
void ProcessorNode::on_sensor_data(const std_msgs::msg::Float64 & msg)
{
    // publisher_ 在 Inactive 状态下存在但未激活，此处主动过滤
    if (!publisher_ || !publisher_->is_activated()) {
        return;
    }

    const int window_size = this->get_parameter("window_size").as_int();
    window_.push_back(msg.data);
    if (static_cast<int>(window_.size()) > window_size) {
        window_.pop_front();
    }

    const double avg = std::accumulate(window_.begin(), window_.end(), 0.0)
                       / static_cast<double>(window_.size());

    std_msgs::msg::Float64 out;
    out.data = avg;
    publisher_->publish(out);

    RCLCPP_DEBUG(get_logger(),
        "[ProcessorNode] raw=%.4f  avg(window=%d)=%.4f",
        msg.data, static_cast<int>(window_.size()), avg);
}

/**
 * @brief 清理订阅、发布器与窗口缓存。
 */
void ProcessorNode::cleanup_resources()
{
    subscription_.reset();
    publisher_.reset();
    window_.clear();
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::ProcessorNode)
