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
 * @param[in] options ROS 2 节点选项。
 */
ProcessorNode::ProcessorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("processor_node", options)
{
    declare_parameters();
    RCLCPP_INFO(get_logger(), "[ProcessorNode] Unconfigured — 节点已构造，等待 configure 指令");
}

/**
 * @brief 声明处理节点使用的全部参数。
 * @return 无返回值。
 */
void ProcessorNode::declare_parameters()
{
    declare_configure_behavior_parameter();
    declare_window_size_parameter();
}

/**
 * @brief 声明 configure_behavior 参数。
 * @return 无返回值。
 */
void ProcessorNode::declare_configure_behavior_parameter()
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description =
        "configure 阶段的行为注入：\n"
        "  'success' — 正常完成（默认）\n"
        "  'failure' — 返回 FAILURE，节点回到 Unconfigured\n"
        "  'error'   — 返回 ERROR，节点进入 ErrorProcessing，触发 on_error";
    this->declare_parameter("configure_behavior", "success", desc);
}

/**
 * @brief 声明 window_size 参数及约束范围。
 * @return 无返回值。
 */
void ProcessorNode::declare_window_size_parameter()
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "移动平均窗口大小，范围 [1, 20]";

    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 20;
    range.step = 1;
    desc.integer_range.push_back(range);

    this->declare_parameter("window_size", 5, desc);
}

/**
 * @brief 生命周期回调：Unconfigured -> Inactive（或失败/错误分支）。
 * @param[in] state 当前生命周期状态（未使用）。
 * @return SUCCESS/FAILURE/ERROR，取决于 configure_behavior 参数。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    const std::string behavior = get_configure_behavior();
    const CallbackReturn configure_result = evaluate_configure_behavior(behavior);
    if (configure_result != CallbackReturn::SUCCESS) {
        return configure_result;
    }

    create_processing_interfaces();

    RCLCPP_INFO(get_logger(),
        "[ProcessorNode] on_configure ✓  subscription + publisher 已创建");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 读取 configure_behavior 参数值。
 * @return configure 阶段行为字符串。
 */
std::string ProcessorNode::get_configure_behavior() const
{
    return this->get_parameter("configure_behavior").as_string();
}

/**
 * @brief 根据 configure_behavior 执行失败/错误分支判断。
 * @param[in] behavior configure 行为参数值。
 * @return SUCCESS 表示继续配置，FAILURE/ERROR 表示中止配置并返回对应状态。
 */
ProcessorNode::CallbackReturn
ProcessorNode::evaluate_configure_behavior(const std::string & behavior) const
{
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

    return CallbackReturn::SUCCESS;
}

/**
 * @brief 创建订阅器、发布器并初始化内部窗口。
 * @return 无返回值。
 */
void ProcessorNode::create_processing_interfaces()
{
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "sensor_data", 10,
        std::bind(&ProcessorNode::on_sensor_data, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_data", 10);
    window_.clear();
}

/**
 * @brief 生命周期回调：Inactive -> Active。
 * @param[in] state 当前生命周期状态。
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
 * @param[in] state 当前生命周期状态。
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
 * @param[in] state 当前生命周期状态（未使用）。
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
 * @param[in] state 触发 shutdown 前的状态。
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
 * @param[in] state 触发错误前的状态。
 * @return SUCCESS 表示错误已处理并允许恢复。
 */
ProcessorNode::CallbackReturn
ProcessorNode::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_ERROR(get_logger(),
        "[ProcessorNode] on_error  触发！前一状态='%s'  正在清理并尝试恢复...",
        state.label().c_str());

    cleanup_resources();

    RCLCPP_WARN(get_logger(),
        "[ProcessorNode] on_error  恢复成功 → 节点将回到 Unconfigured。"
        "  可以重新 configure（修改参数后）");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 判断发布器是否可用于发布。
 * @return true 表示发布器存在且已激活，否则 false。
 */
bool ProcessorNode::is_publisher_ready() const
{
    return publisher_ && publisher_->is_activated();
}

/**
 * @brief 读取 window_size 参数值。
 * @return 当前移动平均窗口大小。
 */
int ProcessorNode::get_window_size() const
{
    return this->get_parameter("window_size").as_int();
}

/**
 * @brief 向窗口加入新样本并按窗口大小截断。
 * @param[in] value 新的传感器样本值。
 * @param[in] window_size 目标窗口大小。
 * @return 无返回值。
 */
void ProcessorNode::update_window(double value, int window_size)
{
    window_.push_back(value);
    if (static_cast<int>(window_.size()) > window_size) {
        window_.pop_front();
    }
}

/**
 * @brief 计算当前窗口平均值。
 * @return 当前窗口平均值。
 */
double ProcessorNode::compute_window_average() const
{
    return std::accumulate(window_.begin(), window_.end(), 0.0)
           / static_cast<double>(window_.size());
}

/**
 * @brief 发布处理后的均值消息。
 * @param[in] average 当前窗口平均值。
 * @return 无返回值。
 */
void ProcessorNode::publish_processed_value(double average)
{
    std_msgs::msg::Float64 out;
    out.data = average;
    publisher_->publish(out);
}

/**
 * @brief 输出处理流程调试日志。
 * @param[in] raw_value 原始传感器值。
 * @param[in] average 当前窗口平均值。
 * @return 无返回值。
 */
void ProcessorNode::log_processing_debug(double raw_value, double average) const
{
    RCLCPP_DEBUG(
        get_logger(),
        "[ProcessorNode] raw=%.4f  avg(window=%d)=%.4f",
        raw_value,
        static_cast<int>(window_.size()),
        average);
}

/**
 * @brief 订阅回调，计算窗口均值并在 Active 状态下发布。
 * @param[in] msg 传感器输入数据。
 * @return 无返回值。
 */
void ProcessorNode::on_sensor_data(const std_msgs::msg::Float64 & msg)
{
    if (!is_publisher_ready()) {
        return;
    }

    const int window_size = get_window_size();
    update_window(msg.data, window_size);

    const double average = compute_window_average();
    publish_processed_value(average);
    log_processing_debug(msg.data, average);
}

/**
 * @brief 清理订阅、发布器与窗口缓存。
 * @return 无返回值。
 */
void ProcessorNode::cleanup_resources()
{
    subscription_.reset();
    publisher_.reset();
    window_.clear();
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::ProcessorNode)
