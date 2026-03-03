#include "ros2_learning_lifecycle/sensor_node.hpp"

/**
 * @file sensor_node.cpp
 * @brief SensorNode 生命周期回调实现。
 */

#include <chrono>
#include <cmath>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace ros2_learning_lifecycle
{

/**
 * @brief 构造节点并声明参数，不创建业务 ROS 资源。
 * @param[in] options ROS 2 节点选项。
 */
SensorNode::SensorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_node", options)
{
    // 在构造阶段集中声明参数，确保后续生命周期回调读取参数时参数接口已就绪。
    declare_parameters();
    // 输出初始化状态日志，明确节点当前仅完成构造、尚未进入配置态。
    RCLCPP_INFO(get_logger(), "[SensorNode] Unconfigured — 节点已构造，等待 configure 指令");
}

/**
 * @brief 声明传感器节点使用的全部参数。
 */
void SensorNode::declare_parameters()
{
    // 先声明传感器标识参数，用于后续日志追踪和多实例区分。
    declare_sensor_id_parameter();
    // 再声明发布频率参数，为定时器周期计算提供可配置输入。
    declare_publish_rate_parameter();
}

/**
 * @brief 声明 sensor_id 参数。
 */
void SensorNode::declare_sensor_id_parameter()
{
    // 构造参数描述符，给参数提供可读说明，方便工具和运维查看含义。
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "传感器编号，用于日志标识";
    // 声明带默认值的 sensor_id，保证节点在未外部传参时也可正常启动。
    this->declare_parameter("sensor_id", "sensor_0", desc);
}

/**
 * @brief 声明 publish_rate_hz 参数及约束范围。
 */
void SensorNode::declare_publish_rate_parameter()
{
    // 构造参数描述符，为发布频率参数附加语义说明与约束信息。
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "数据发布频率（Hz），范围 [1, 10]";

    // 配置整数范围约束，防止非法频率导致定时器周期异常。
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 10;
    range.step = 1;
    // 将范围约束写入描述符，使 ROS 参数系统在设置时可进行校验。
    desc.integer_range.push_back(range);

    // 声明 publish_rate_hz 并设置默认值 2Hz，确保未配置时也有稳定行为。
    this->declare_parameter("publish_rate_hz", 2, desc);
}

/**
 * @brief 生命周期回调：Unconfigured -> Inactive。
 * @param[in] state 当前生命周期状态（未使用）。
 * @return SUCCESS 表示发布器创建成功。
 */
SensorNode::CallbackReturn
SensorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    // 在配置阶段创建发布器资源，但此时仍保持未激活，符合生命周期语义。
    create_sensor_publisher();

    // 记录配置完成日志，并带上 sensor_id 便于多节点场景定位实例。
    RCLCPP_INFO(
        get_logger(),
        "[SensorNode] on_configure ✓  sensor_id='%s'  publisher 已创建（未激活）",
        get_sensor_id().c_str());
    // 返回成功，允许状态机从 Unconfigured 进入 Inactive。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 创建 /sensor_data 生命周期发布器。
 */
void SensorNode::create_sensor_publisher()
{
    // 创建生命周期发布器并绑定到 sensor_data 话题，队列深度 10 平衡实时性与缓冲能力。
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sensor_data", 10);
}

/**
 * @brief 生命周期回调：Inactive -> Active。
 * @param[in] state 当前生命周期状态。
 * @return SUCCESS 表示已激活发布器并启动定时器。
 */
SensorNode::CallbackReturn
SensorNode::on_activate(const rclcpp_lifecycle::State & state)
{
    // 必须先调用父类，激活所有 LifecyclePublisher
    LifecycleNode::on_activate(state);

    // 读取当前频率参数，确保定时器按最新配置运行。
    const int rate_hz = get_publish_rate_hz();
    // 基于频率创建定时器，驱动周期性发布链路开始工作。
    create_publish_timer(rate_hz);

    // 输出激活完成日志，明确发布器与定时器均已生效。
    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_activate ✓  publisher 已激活，timer 已创建（%d Hz）", rate_hz);
    // 返回成功，允许状态机保持在 Active。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 创建定时发布用 wall timer。
 * @param[in] rate_hz 发布频率（Hz）。
 */
void SensorNode::create_publish_timer(int rate_hz)
{
    // 将 Hz 频率转换为毫秒周期，供 wall timer 使用。
    const auto period = std::chrono::milliseconds(1000 / rate_hz);
    // 创建周期定时器并绑定 on_timer 回调，实现固定频率的数据发布触发。
    timer_ = this->create_wall_timer(period, std::bind(&SensorNode::on_timer, this));
}

/**
 * @brief 读取 publish_rate_hz 参数值。
 * @return 当前发布频率（Hz）。
 */
int SensorNode::get_publish_rate_hz() const
{
    // 从参数服务器读取发布频率，保证运行期可通过参数机制动态调整。
    return this->get_parameter("publish_rate_hz").as_int();
}

/**
 * @brief 读取 sensor_id 参数值。
 * @return 当前传感器编号字符串。
 */
std::string SensorNode::get_sensor_id() const
{
    // 从参数服务器读取传感器标识，用于日志标注和实例识别。
    return this->get_parameter("sensor_id").as_string();
}

/**
 * @brief 生命周期回调：Active -> Inactive。
 * @param[in] state 当前生命周期状态。
 * @return SUCCESS 表示已停止定时器并停用发布器。
 */
SensorNode::CallbackReturn
SensorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    timer_.reset();  // 停止定时器，不再触发 on_timer

    // 再调用父类，停用所有 LifecyclePublisher
    LifecycleNode::on_deactivate(state);

    // 记录停用结果，确认发布触发源和发布通道均已停止。
    RCLCPP_INFO(get_logger(), "[SensorNode] on_deactivate ✓  timer 已销毁，publisher 已停用");
    // 返回成功，允许状态机从 Active 回到 Inactive。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期回调：Inactive -> Unconfigured。
 * @param[in] state 当前生命周期状态（未使用）。
 * @return SUCCESS 表示资源已清理。
 */
SensorNode::CallbackReturn
SensorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    // 在清理阶段释放发布器资源，避免无效句柄跨状态残留。
    publisher_.reset();
    // 记录清理完成，便于确认节点已回到可重新配置的初始状态。
    RCLCPP_INFO(get_logger(), "[SensorNode] on_cleanup ✓  publisher 已销毁，回到 Unconfigured");
    // 返回成功，允许状态机完成 Inactive -> Unconfigured 转换。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期回调：Any -> Finalized。
 * @param[in] state 触发 shutdown 前的状态。
 * @return SUCCESS 表示已完成终态清理。
 */
SensorNode::CallbackReturn
SensorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    // 输出关闭来源状态，帮助排查是从哪个生命周期阶段触发的 shutdown。
    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_shutdown  从状态 '%s' 关闭", state.label().c_str());
    // 先停定时器，避免关闭过程中仍有回调并发访问资源。
    timer_.reset();
    // 再释放发布器，完成终态资源回收。
    publisher_.reset();
    // 返回成功，告知生命周期框架终态清理已完成。
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 构造模拟传感器输出消息并推进内部相位。
 * @return 待发布的 Float64 消息。
 */
std_msgs::msg::Float64 SensorNode::build_sensor_message()
{
    // 推进内部相位变量，模拟连续时间序列下的传感器变化。
    simulated_value_ += 0.1;

    // 构造 ROS 消息对象，封装本次发布的数据。
    std_msgs::msg::Float64 msg;
    // 使用正弦函数生成平滑的模拟测量值，便于下游观察周期信号。
    msg.data = std::sin(simulated_value_);
    // 返回完整消息供发布流程直接使用。
    return msg;
}

/**
 * @brief 输出发布调试日志。
 * @param[in] value 本次发布的数据值。
 */
void SensorNode::log_publish_debug(double value) const
{
    // 输出调试级日志，记录传感器标识与当前数值，便于低成本追踪发布行为。
    RCLCPP_DEBUG(
        get_logger(),
        "[SensorNode] 发布 sensor_id='%s'  data=%.4f",
        get_sensor_id().c_str(),
        value);
}

/**
 * @brief 定时器回调，发布正弦波模拟传感器数据。
 */
void SensorNode::on_timer()
{
    // 先生成本周期的模拟传感器消息，保证数据与内部状态同步推进。
    const auto msg = build_sensor_message();
    // 通过已激活的生命周期发布器发送数据到 sensor_data 话题。
    publisher_->publish(msg);
    // 发布后记录调试信息，便于核对发送值与发布节奏。
    log_publish_debug(msg.data);
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::SensorNode)
