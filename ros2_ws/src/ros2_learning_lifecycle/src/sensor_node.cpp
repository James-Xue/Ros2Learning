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
    declare_parameters();
    RCLCPP_INFO(get_logger(), "[SensorNode] Unconfigured — 节点已构造，等待 configure 指令");
}

/**
 * @brief 声明传感器节点使用的全部参数。
 * @return 无返回值。
 */
void SensorNode::declare_parameters()
{
    declare_sensor_id_parameter();
    declare_publish_rate_parameter();
}

/**
 * @brief 声明 sensor_id 参数。
 * @return 无返回值。
 */
void SensorNode::declare_sensor_id_parameter()
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "传感器编号，用于日志标识";
    this->declare_parameter("sensor_id", "sensor_0", desc);
}

/**
 * @brief 声明 publish_rate_hz 参数及约束范围。
 * @return 无返回值。
 */
void SensorNode::declare_publish_rate_parameter()
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "数据发布频率（Hz），范围 [1, 10]";

    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 10;
    range.step = 1;
    desc.integer_range.push_back(range);

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
    create_sensor_publisher();

    RCLCPP_INFO(
        get_logger(),
        "[SensorNode] on_configure ✓  sensor_id='%s'  publisher 已创建（未激活）",
        get_sensor_id().c_str());
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 创建 /sensor_data 生命周期发布器。
 * @return 无返回值。
 */
void SensorNode::create_sensor_publisher()
{
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

    const int rate_hz = get_publish_rate_hz();
    create_publish_timer(rate_hz);

    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_activate ✓  publisher 已激活，timer 已创建（%d Hz）", rate_hz);
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 创建定时发布用 wall timer。
 * @param[in] rate_hz 发布频率（Hz）。
 * @return 无返回值。
 */
void SensorNode::create_publish_timer(int rate_hz)
{
    const auto period = std::chrono::milliseconds(1000 / rate_hz);
    timer_ = this->create_wall_timer(period, std::bind(&SensorNode::on_timer, this));
}

/**
 * @brief 读取 publish_rate_hz 参数值。
 * @return 当前发布频率（Hz）。
 */
int SensorNode::get_publish_rate_hz() const
{
    return this->get_parameter("publish_rate_hz").as_int();
}

/**
 * @brief 读取 sensor_id 参数值。
 * @return 当前传感器编号字符串。
 */
std::string SensorNode::get_sensor_id() const
{
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

    RCLCPP_INFO(get_logger(), "[SensorNode] on_deactivate ✓  timer 已销毁，publisher 已停用");
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
    publisher_.reset();
    RCLCPP_INFO(get_logger(), "[SensorNode] on_cleanup ✓  publisher 已销毁，回到 Unconfigured");
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
    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_shutdown  从状态 '%s' 关闭", state.label().c_str());
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 构造模拟传感器输出消息并推进内部相位。
 * @return 待发布的 Float64 消息。
 */
std_msgs::msg::Float64 SensorNode::build_sensor_message()
{
    simulated_value_ += 0.1;

    std_msgs::msg::Float64 msg;
    msg.data = std::sin(simulated_value_);
    return msg;
}

/**
 * @brief 输出发布调试日志。
 * @param[in] value 本次发布的数据值。
 * @return 无返回值。
 */
void SensorNode::log_publish_debug(double value) const
{
    RCLCPP_DEBUG(
        get_logger(),
        "[SensorNode] 发布 sensor_id='%s'  data=%.4f",
        get_sensor_id().c_str(),
        value);
}

/**
 * @brief 定时器回调，发布正弦波模拟传感器数据。
 * @return 无返回值。
 */
void SensorNode::on_timer()
{
    const auto msg = build_sensor_message();
    publisher_->publish(msg);
    log_publish_debug(msg.data);
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::SensorNode)
