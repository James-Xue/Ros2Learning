#include "ros2_learning_qos/mission_control_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace ros2_learning_qos
{

MissionControlNode::MissionControlNode() : Node("mission_control")
{
  // 1. 订阅视频 (Best Effort) -> 兼容
  auto camera_qos = rclcpp::SensorDataQoS();
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", camera_qos,
    [](const sensor_msgs::msg::Image::SharedPtr) {
      // 收到视频帧，不做打印以免刷屏
    });

  // 2. 订阅指令确认 (Reliable) -> 兼容
  auto cmd_ack_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  cmd_ack_sub_ = this->create_subscription<std_msgs::msg::String>(
    "rover/cmd_ack", cmd_ack_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "[Control] 收到指令确认: %s", msg->data.c_str());
    });

  // 3. 订阅地图 (Transient Local) -> 兼容
  // 注意：即使发布者早就发完了，只要它是 Transient Local，我们也能收到
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_sub_ = this->create_subscription<std_msgs::msg::String>(
    "rover/map", map_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), ">>> [Control] 同步到地图数据: %s <<<", msg->data.c_str());
    });

  // 4. 订阅心跳 (Deadline 监控)
  auto heartbeat_qos = rclcpp::QoS(rclcpp::KeepLast(10)).deadline(600ms);

  // 设置事件回调
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo & event) {
      RCLCPP_ERROR(this->get_logger(),
        "!!! [ALARM] 心跳丢失！截止期未收到数据 (Deadline Missed). Count: %d !!!",
        event.total_count);
    };

  heartbeat_sub_ = this->create_subscription<std_msgs::msg::String>(
    "rover/heartbeat", heartbeat_qos,
    [this](const std_msgs::msg::String::SharedPtr) {
      // 正常收到心跳
      // RCLCPP_INFO(this->get_logger(), "Heartbeat OK");
    },
    sub_options);

  // 5. [测试不兼容 QoS]
  // 尝试用 Reliable 订阅 Best Effort 的发布者 -> 应该收不到
  auto incompatible_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // 注册不兼容 QoS 事件回调
  rclcpp::SubscriptionOptions incompatible_opts;
  // 使用新版 API 回调名称 (Jazzy/Rolling)
  incompatible_opts.event_callbacks.incompatible_qos_callback =
    [this](rclcpp::QOSRequestedIncompatibleQoSInfo & event) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [Control] QoS 不兼容! Policy ID: %d, 累计次数: %d",
        event.last_policy_kind, event.total_count);
    };

  incompatible_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", incompatible_qos, // Pub 是 BestEffort, Sub 是 Reliable
    [this](const sensor_msgs::msg::Image::SharedPtr) {
      RCLCPP_ERROR(this->get_logger(), "如果不兼容你还能收到，那就是见鬼了！");
    },
    incompatible_opts);
}

} // namespace ros2_learning_qos
