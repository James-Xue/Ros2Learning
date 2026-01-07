// 防止头文件被多次包含
#pragma once

// 标准库头：chrono 用于时长类型（如 100ms, 1s），memory 用于智能指针
#include <chrono>
#include <memory>

// ROS 消息类型：目标位姿和带协方差的位姿（用于 initialpose）
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// Nav2 提供的 NavigateToPose action 定义
#include "nav2_msgs/action/navigate_to_pose.hpp"
// rclcpp 核心头文件（节点、日志等）
#include "rclcpp/rclcpp.hpp"
// rclcpp action 客户端的封装
#include "rclcpp_action/rclcpp_action.hpp"
// tf2 四元数数学类型及与 geometry_msgs 的互转支持
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 定义一个用于与 Nav2 交互的客户端类，继承自 rclcpp::Node
class Nav2Client : public rclcpp::Node
{
  public:
    // 为常用类型起别名，便于在实现文件中书写
    // NavigateToPose: Nav2 的导航目标 action 类型
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    // GoalHandle: action 客户端返回的目标句柄类型，用于取消或获取结果
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    // PoseWithCovarianceStamped: 带协方差的位姿消息（常用于 initialpose）
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    // PoseStamped: 目标位姿的消息类型
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    // NavigateClient: action 客户端类型（模板化为 NavigateToPose）
    using NavigateClient = rclcpp_action::Client<NavigateToPose>;
    // PosePublisher: 发布 PoseWithCovarianceStamped 的发布器类型
    using PosePublisher = rclcpp::Publisher<PoseWithCovarianceStamped>;

    // 构造函数：在实现中会初始化发布器和 action 客户端
    Nav2Client();

    // 运行入口：执行等待、发布 initial pose、发送目标并处理结果
    void run();

  private:
    // 等待 Nav2 action server 可用
    bool wait_for_server();
    // 等待仿真时间或系统时间可用（use_sim_time 情况）
    void wait_for_time();
    // 等待 TF 中 map->base_link 变换可用
    void wait_for_tf();
    // 发布初始位姿到 /initialpose
    void publish_initial_pose();
    // 发送导航目标，返回 GoalHandle（失败时返回 nullptr）
    GoalHandle::SharedPtr send_goal();

    // 成员变量：发布 initial pose 的发布器
    PosePublisher::SharedPtr mInitialPosePublisher;
    // 成员变量：Nav2 的 action 客户端
    NavigateClient::SharedPtr mActionClient;
};
