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
    // 在节点构造阶段统一声明参数，确保后续生命周期回调读取参数时一定存在默认值与约束。
    declare_parameters();
    // 立即输出当前生命周期位置，帮助调试时确认节点已创建但尚未进入 configure。
    RCLCPP_INFO(get_logger(), "[ProcessorNode] Unconfigured — 节点已构造，等待 configure 指令");
}

/**
 * @brief 声明处理节点使用的全部参数。
 */
void ProcessorNode::declare_parameters()
{
    // 分步骤声明参数，便于按功能扩展并保持每个声明函数职责单一。
    declare_configure_behavior_parameter();
    // 将窗口参数与行为注入参数分离，减少后续维护时的耦合风险。
    declare_window_size_parameter();
}

/**
 * @brief 声明 configure_behavior 参数。
 */
void ProcessorNode::declare_configure_behavior_parameter()
{
    // 构造参数描述符用于在 `ros2 param` 等工具中提供可读说明，便于学习与调试。
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description =
        "configure 阶段的行为注入：\n"
        "  'success' — 正常完成（默认）\n"
        "  'failure' — 返回 FAILURE，节点回到 Unconfigured\n"
        "  'error'   — 返回 ERROR，节点进入 ErrorProcessing，触发 on_error";
    // 声明带默认值的行为注入参数，默认走成功路径，避免首次启动即进入异常分支。
    this->declare_parameter("configure_behavior", "success", desc);
}

/**
 * @brief 声明 window_size 参数及约束范围。
 */
void ProcessorNode::declare_window_size_parameter()
{
    // 先准备参数描述，向使用者明确该参数控制移动平均窗口大小。
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "移动平均窗口大小，范围 [1, 20]";

    // 通过整数范围约束防止非法窗口值，避免运行时出现空窗口或超大窗口带来的异常行为。
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 20;
    range.step = 1;
    // 将范围约束写入描述符，使参数服务端在设置参数时可进行校验。
    desc.integer_range.push_back(range);

    // 声明窗口参数并给出学习场景下较稳妥的默认值，兼顾平滑性与响应速度。
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
    // 读取行为注入参数，用于决定本次 configure 是成功、失败还是错误分支。
    const std::string behavior = get_configure_behavior();
    // 统一在独立函数中评估分支，保持 on_configure 主流程清晰。
    const CallbackReturn configure_result = evaluate_configure_behavior(behavior);
    // 若注入了 failure/error，立即返回对应状态，避免继续创建通信接口。
    if (configure_result != CallbackReturn::SUCCESS) {
        return configure_result;
    }

    // 仅在 configure 成功路径创建订阅与发布接口，保证生命周期语义正确。
    create_processing_interfaces();

    // 打印配置成功日志，便于确认节点已经完成资源初始化。
    RCLCPP_INFO(get_logger(),
        "[ProcessorNode] on_configure ✓  subscription + publisher 已创建");
    // 返回 SUCCESS 让节点从 Unconfigured 进入 Inactive。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 读取 configure_behavior 参数值。
 * @return configure 阶段行为字符串。
 */
std::string ProcessorNode::get_configure_behavior() const
{
    // 从参数服务器读取 configure 行为字符串，供生命周期分支判断使用。
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
    // 显式模拟 configure 失败分支，用于演示节点如何回退到 Unconfigured。
    if (behavior == "failure") {
        RCLCPP_WARN(get_logger(),
            "[ProcessorNode] on_configure → FAILURE（注入）"
            "  节点将回到 Unconfigured，不进入 ErrorProcessing");
        return CallbackReturn::FAILURE;
    }

    // 显式模拟 configure 错误分支，用于触发 ErrorProcessing 与 on_error 恢复流程。
    if (behavior == "error") {
        RCLCPP_ERROR(get_logger(),
            "[ProcessorNode] on_configure → ERROR（注入）"
            "  节点将进入 ErrorProcessing，触发 on_error 回调");
        return CallbackReturn::ERROR;
    }

    // 未命中注入分支时按正常流程返回 SUCCESS。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 创建订阅器、发布器并初始化内部窗口。
 */
void ProcessorNode::create_processing_interfaces()
{
    // 创建输入订阅器，将传感器数据绑定到处理回调，形成处理链入口。
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "sensor_data", 10,
        std::bind(&ProcessorNode::on_sensor_data, this, std::placeholders::_1));

    // 创建生命周期发布器，后续仅在 Active 状态发布处理结果。
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_data", 10);
    // 每次 configure 重新初始化窗口缓存，避免旧周期残留数据影响当前结果。
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
    // 调用基类激活逻辑，真正启用生命周期发布器。
    LifecycleNode::on_activate(state);
    // 记录进入 Active 的时间点，便于确认何时开始对外发布结果。
    RCLCPP_INFO(get_logger(), "[ProcessorNode] on_activate ✓  publisher 已激活，开始处理数据");
    // 返回 SUCCESS 维持生命周期状态迁移。
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
    // 调用基类停用逻辑，使发布器进入不可发布状态。
    LifecycleNode::on_deactivate(state);
    // 输出停用日志，便于排查“有输入但无输出”是否由生命周期状态导致。
    RCLCPP_INFO(get_logger(), "[ProcessorNode] on_deactivate ✓  publisher 已停用，停止发布结果");
    // 返回 SUCCESS 确认停用流程完成。
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
    // 在 cleanup 阶段统一释放通信对象和缓存，恢复到可重新 configure 的干净状态。
    cleanup_resources();
    // 记录资源清理结果，便于验证状态机回退行为是否符合预期。
    RCLCPP_INFO(get_logger(), "[ProcessorNode] on_cleanup ✓  资源已释放，回到 Unconfigured");
    // 返回 SUCCESS 允许状态机完成 Inactive -> Unconfigured。
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
    // 输出关闭前状态，帮助定位节点是从哪条生命周期路径进入 shutdown。
    RCLCPP_INFO(get_logger(),
        "[ProcessorNode] on_shutdown  从状态 '%s' 关闭", state.label().c_str());
    // 在终止前执行与 cleanup 一致的资源释放，防止句柄与缓存残留。
    cleanup_resources();
    // 返回 SUCCESS 表示终态收尾完成。
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
    // 先记录错误来源状态，便于追溯是哪次状态迁移触发了 ErrorProcessing。
    RCLCPP_ERROR(get_logger(),
        "[ProcessorNode] on_error  触发！前一状态='%s'  正在清理并尝试恢复...",
        state.label().c_str());

    // 出错时优先清理资源，避免带着异常状态继续运行导致二次故障。
    cleanup_resources();

    // 明确告知已恢复到可重新 configure 的路径，指导下一步人工操作。
    RCLCPP_WARN(get_logger(),
        "[ProcessorNode] on_error  恢复成功 → 节点将回到 Unconfigured。"
        "  可以重新 configure（修改参数后）");
    // 返回 SUCCESS 让生命周期框架执行错误恢复到 Unconfigured。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 判断发布器是否可用于发布。
 * @return true 表示发布器存在且已激活，否则 false。
 */
bool ProcessorNode::is_publisher_ready() const
{
    // 仅当发布器对象存在且已激活时才允许发布，防止在非 Active 状态误发消息。
    return publisher_ && publisher_->is_activated();
}

/**
 * @brief 读取 window_size 参数值。
 * @return 当前移动平均窗口大小。
 */
int ProcessorNode::get_window_size() const
{
    // 每次处理前实时读取参数，支持运行时动态调整窗口大小。
    return this->get_parameter("window_size").as_int();
}

/**
 * @brief 向窗口加入新样本并按窗口大小截断。
 * @param[in] value 新的传感器样本值。
 * @param[in] window_size 目标窗口大小。
 */
void ProcessorNode::update_window(double value, int window_size)
{
    // 先追加最新样本，确保平均值始终包含当前时刻输入。
    window_.push_back(value);
    // 当样本数超过窗口上限时丢弃最旧值，维持固定长度的滑动窗口语义。
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
    // 对窗口样本求和并除以样本数，得到当前移动平均输出值。
    return std::accumulate(window_.begin(), window_.end(), 0.0)
           / static_cast<double>(window_.size());
}

/**
 * @brief 发布处理后的均值消息。
 * @param[in] average 当前窗口平均值。
 */
void ProcessorNode::publish_processed_value(double average)
{
    // 组装标准浮点消息，保持与输入输出话题类型一致，便于链路复用。
    std_msgs::msg::Float64 out;
    out.data = average;
    // 通过生命周期发布器发出处理结果（仅在 Active 状态可成功发布）。
    publisher_->publish(out);
}

/**
 * @brief 输出处理流程调试日志。
 * @param[in] raw_value 原始传感器值。
 * @param[in] average 当前窗口平均值。
 */
void ProcessorNode::log_processing_debug(double raw_value, double average) const
{
    // 以 DEBUG 级别输出原始值与窗口均值，用于调试滤波效果而不污染常规日志。
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
 */
void ProcessorNode::on_sensor_data(const std_msgs::msg::Float64 & msg)
{
    // 若当前不处于可发布状态则直接返回，保证生命周期约束优先于数据处理。
    if (!is_publisher_ready()) {
        return;
    }

    // 读取当前窗口参数，使处理逻辑可在运行期随参数变化即时生效。
    const int window_size = get_window_size();
    // 将新样本纳入滑动窗口并执行必要截断。
    update_window(msg.data, window_size);

    // 计算更新后窗口的平均值，作为本次输出结果。
    const double average = compute_window_average();
    // 发布处理结果到下游话题。
    publish_processed_value(average);
    // 输出调试日志，便于观察输入与输出的对应关系。
    log_processing_debug(msg.data, average);
}

/**
 * @brief 清理订阅、发布器与窗口缓存。
 */
void ProcessorNode::cleanup_resources()
{
    // 释放订阅器句柄，停止接收新数据并断开相关回调链路。
    subscription_.reset();
    // 释放发布器句柄，确保离开活动流程后不再持有发布资源。
    publisher_.reset();
    // 清空窗口缓存，避免旧数据污染下一次 configure 后的计算结果。
    window_.clear();
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::ProcessorNode)
