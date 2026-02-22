#ifndef ROS2_LEARNING_QOS_NODES_HPP
#define ROS2_LEARNING_QOS_NODES_HPP

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

// 注意：不在头文件中使用 using namespace，避免污染包含此头文件的翻译单元

namespace ros2_learning_qos
{
using namespace std::chrono_literals; // 限定在本命名空间内

/**
 * @brief 场景：火星探测车 (Mars Rover)
 * 职责：发布不同 QoS 策略的数据
 */
class RoverNode : public rclcpp::Node
{
public:
  RoverNode() : Node("mars_rover")
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

private:
  void timer_callback()
  {
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
    
    // 模拟偶尔的心跳丢失 (用于触发 Deadline Missed 事件)
    // 注意：不使用 sleep_for()，因为在单线程 Executor 中会阻塞整个事件循环
    if (++count_ % 20 == 0) { // 每 10 秒左右
      RCLCPP_WARN(this->get_logger(), "[Rover] 模拟心跳丢失（跳过本次及后续数次发布）！");
      skip_heartbeat_rounds_ = 3; // 跳过 3 次发布 (1.5s > 600ms Deadline)
    }
    if (skip_heartbeat_rounds_ > 0) {
      --skip_heartbeat_rounds_;
      return; // 跳过本次心跳发布，不阻塞其他功能
    }
  }

  void publish_static_map()
  {
    std_msgs::msg::String map_msg;
    map_msg.data = "{Map Data: Crater_A, Rock_B, Base_C}";
    map_pub_->publish(map_msg);
    RCLCPP_INFO(this->get_logger(), "[Rover] 地图数据已发布 (Transient Local). 后来的订阅者也能收到！");
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;                // 定时器回调计数
  int skip_heartbeat_rounds_ = 0; // 跳过心跳的剩余轮数
};

/**
 * @brief 场景：地球控制中心 (Mission Control)
 * 职责：订阅数据，监控 QoS 事件
 */
class MissionControlNode : public rclcpp::Node
{
public:
  MissionControlNode() : Node("mission_control")
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
    
    // 注册不兼容 QoS 事件回调，当 DDS 检测到 Pub/Sub 的 QoS 不兼容时，会触发此回调
    rclcpp::SubscriptionOptions incompatible_opts;
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

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr incompatible_sub_;
};

} // namespace ros2_learning_qos

#endif // ROS2_LEARNING_QOS_NODES_HPP
