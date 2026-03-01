#include "ros2_learning_lifecycle/sensor_node.hpp"

#include <chrono>
#include <cmath>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace ros2_learning_lifecycle
{

SensorNode::SensorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sensor_node", options)
{
    // 声明参数 — 构造时（Unconfigured 状态）只做参数声明，不创建任何 ROS 资源
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "传感器编号，用于日志标识";
        this->declare_parameter("sensor_id", "sensor_0", desc);
    }
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "数据发布频率（Hz），范围 [1, 10]";
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = 1;
        range.to_value   = 10;
        range.step       = 1;
        desc.integer_range.push_back(range);
        this->declare_parameter("publish_rate_hz", 2, desc);
    }

    RCLCPP_INFO(get_logger(), "[SensorNode] Unconfigured — 节点已构造，等待 configure 指令");
}

// ──────────────────────────────────────────────────────────────
// on_configure: Unconfigured → Inactive
// 只创建 publisher，不创建 timer（timer 在 on_activate 创建）
// ──────────────────────────────────────────────────────────────
SensorNode::CallbackReturn
SensorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sensor_data", 10);

    const auto sensor_id = this->get_parameter("sensor_id").as_string();
    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_configure ✓  sensor_id='%s'  publisher 已创建（未激活）",
        sensor_id.c_str());
    return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_activate: Inactive → Active
// 先激活 publisher（父类调用），再创建 timer
// ──────────────────────────────────────────────────────────────
SensorNode::CallbackReturn
SensorNode::on_activate(const rclcpp_lifecycle::State & state)
{
    // 必须先调用父类，激活所有 LifecyclePublisher
    LifecycleNode::on_activate(state);

    const int rate_hz = this->get_parameter("publish_rate_hz").as_int();
    const auto period = std::chrono::milliseconds(1000 / rate_hz);
    timer_ = this->create_wall_timer(period, std::bind(&SensorNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_activate ✓  publisher 已激活，timer 已创建（%d Hz）", rate_hz);
    return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_deactivate: Active → Inactive
// 先销毁 timer（停止定时），再停用 publisher（父类调用）
// ──────────────────────────────────────────────────────────────
SensorNode::CallbackReturn
SensorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    timer_.reset();  // 停止定时器，不再触发 on_timer

    // 再调用父类，停用所有 LifecyclePublisher
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(get_logger(), "[SensorNode] on_deactivate ✓  timer 已销毁，publisher 已停用");
    return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_cleanup: Inactive → Unconfigured
// 释放 publisher（timer 在 on_deactivate 已销毁）
// ──────────────────────────────────────────────────────────────
SensorNode::CallbackReturn
SensorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    publisher_.reset();
    RCLCPP_INFO(get_logger(), "[SensorNode] on_cleanup ✓  publisher 已销毁，回到 Unconfigured");
    return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_shutdown: Any → Finalized
// 无论从哪个状态 shutdown，都清理所有资源
// ──────────────────────────────────────────────────────────────
SensorNode::CallbackReturn
SensorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(),
        "[SensorNode] on_shutdown  从状态 '%s' 关闭", state.label().c_str());
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_timer: 仅在 Active 状态下被调用（因为 timer 在 on_activate 创建）
// 发布模拟传感器读数（正弦波）
// ──────────────────────────────────────────────────────────────
void SensorNode::on_timer()
{
    simulated_value_ += 0.1;
    std_msgs::msg::Float64 msg;
    msg.data = std::sin(simulated_value_);

    publisher_->publish(msg);

    const auto sensor_id = this->get_parameter("sensor_id").as_string();
    RCLCPP_DEBUG(get_logger(),
        "[SensorNode] 发布 sensor_id='%s'  data=%.4f", sensor_id.c_str(), msg.data);
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::SensorNode)
