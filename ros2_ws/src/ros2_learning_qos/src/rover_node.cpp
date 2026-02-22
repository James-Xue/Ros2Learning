#include "ros2_learning_qos/rover_node.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace ros2_learning_qos
{

RoverNode::RoverNode() : Node("mars_rover")
{
  // 1. [摄像头数据] -> Best Effort (丢包不重发，保证实时性)
  // 模拟高频视频流，允许网络抖动时丢帧
  auto camera_qos = rclcpp::SensorDataQoS(); // 默认就是 Best Effort
  camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", camera_qos);

  // 2. [紧急指令确认] -> Reliable (必须送达)
  // 重要的状态确认，不允许丢失
  auto cmd_ack_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  cmd_ack_pub_ = this->create_publisher<std_msgs::msg::String>("rover/cmd_ack", cmd_ack_qos);

  // 3. [地图数据] -> Transient Local (持久化)
  // 即使订阅者晚加入，也能收到最后一次发布的地图数据（类似 Latched Topic）
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_pub_ = this->create_publisher<std_msgs::msg::String>("rover/map", map_qos);

  // 4. [心跳检测] -> Deadline (截止期)
  // 如果超过一定时间没发布，系统应报警
  // 设置 Deadline 为 600ms，如果我们发布慢了，Subscription 端会收到通知
  auto heartbeat_qos = rclcpp::QoS(rclcpp::KeepLast(10)).deadline(600ms);
  heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>("rover/heartbeat", heartbeat_qos);

  // 定时器模拟数据发送
  timer_ = this->create_wall_timer(500ms, std::bind(&RoverNode::timer_callback, this));

  // 仅发布一次的地图数据 (模拟 Late-Joiner 场景)
  publish_static_map();
}

void RoverNode::timer_callback()
{
  // 模拟偶尔的心跳丢失 (用于触发 Deadline Missed 事件)
  // 注意：不使用 sleep_for()，因为在单线程 Executor 中会阻塞整个事件循环
  if (++count_ % 20 == 0) { // 每 10 秒左右
    RCLCPP_WARN(this->get_logger(), "[Rover] 模拟心跳丢失（跳过本次及后续数次发布）！");
    skip_heartbeat_rounds_ = 3; // 跳过 3 次发布 (1.5s > 600ms Deadline)
  }

  if (skip_heartbeat_rounds_ > 0) {
    --skip_heartbeat_rounds_;
    // 仍然需要发布其他不被"卡顿"影响的数据吗？
    // 在这个模拟中，我们假设是整个系统卡顿，还是仅仅心跳逻辑挂了？
    // 这里选择 skip 整个回调，模拟的是"心跳逻辑"层面的故障，而非系统卡死。
    // 这更符合"不阻塞"的最佳实践。
    return;
  }

  // 发布图像 (Best Effort)
  auto img_msg = sensor_msgs::msg::Image();
  img_msg.header.frame_id = "camera_link";
  img_msg.header.stamp = this->now();
  // 模拟数据...
  camera_pub_->publish(img_msg);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[Rover] 发送视频流 (Best Effort)");

  // 发布指令确认 (Reliable)
  std_msgs::msg::String ack_msg;
  ack_msg.data = "CMD_EXECUTED_OK";
  cmd_ack_pub_->publish(ack_msg);

  // 发布心跳 (Deadline测试)
  std_msgs::msg::String hb_msg;
  hb_msg.data = "ALIVE";
  heartbeat_pub_->publish(hb_msg);
}

void RoverNode::publish_static_map()
{
  std_msgs::msg::String map_msg;
  map_msg.data = "{Map Data: Crater_A, Rock_B, Base_C}";
  map_pub_->publish(map_msg);
  RCLCPP_INFO(this->get_logger(), "[Rover] 地图数据已发布 (Transient Local). 后来的订阅者也能收到！");
}

} // namespace ros2_learning_qos
