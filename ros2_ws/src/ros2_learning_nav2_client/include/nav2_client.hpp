// 防止头文件被重复包含
#pragma once

// 标准库：chrono 用于时间/时长类型（例：100ms、1s），memory 用于智能指针
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

// ROS 消息类型：目标位姿和带协方差的位姿（常用于 initialpose）
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// Nav2 提供的 NavigateToPose action 类型，用于发送导航目标
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
// rclcpp 核心：节点、日志、时钟等功能
#include "rclcpp/rclcpp.hpp"
// rclcpp action 支持库，用于创建 action 客户端
#include "rclcpp_action/rclcpp_action.hpp"
// tf2 四元数和与 geometry_msgs 的互转支持（用于设置朝向）
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Nav2Client 类：封装一个最小的 Nav2 action 客户端，用于发布初始位姿并发送导航目标
class Nav2Client : public rclcpp::Node
{
  public:
    // 常用类型别名，便于在实现文件中简洁书写：
    // NavigateToPose: Nav2 的 action 类型
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    // GoalHandle: action 客户端返回的目标句柄类型，可用于取消或查询状态
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    // PoseWithCovarianceStamped: 带协方差的位姿消息类型（用于 /initialpose）
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    // PoseStamped: 目标位姿的消息类型（用于发送导航目标）
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    // NavigateClient: action 客户端类型别名（模板化为 NavigateToPose）
    using NavigateClient = rclcpp_action::Client<NavigateToPose>;
    // PosePublisher: 发布 PoseWithCovarianceStamped 的发布器类型别名
    using PosePublisher = rclcpp::Publisher<PoseWithCovarianceStamped>;

    // 构造函数：在实现中会初始化节点、发布器和 action 客户端
    Nav2Client();

    // 运行函数：等待时间/TF、发布初始位姿、发送目标并等待结果（实现文件中定义）
    void run();

  private:
    // 等待 Nav2 action server 可用，返回 true/false
    bool wait_for_server();
    // 等待 Nav2 lifecycle manager 报告栈已 active（避免过早发 goal 被拒）
    bool wait_for_nav2_active();
    // 等待时间（use_sim_time 场景中等待时钟非 0）
    bool wait_for_time();
    // 等待 TF（如 map -> base_link）可用，成功返回 true
    bool wait_for_tf();
    // 发布初始位姿到 /initialpose（带协方差）
    void publish_initial_pose();
    // 等待 AMCL 发布位姿（确保 initialpose 已被处理，定位/TF 更稳定）
    bool wait_for_amcl_pose();
    // 发送导航目标并返回 GoalHandle；失败时返回 nullptr
    GoalHandle::SharedPtr send_goal();

    // 让本节点在指定时长内处理回调（订阅/服务响应等）
    void spin_some_for(std::chrono::nanoseconds duration);

    // 成员变量说明：
    // m_InitialPosePublisher: 发布 /initialpose 的发布器，类型为 PoseWithCovarianceStamped
    PosePublisher::SharedPtr m_InitialPosePublisher;
    // m_ActionClient: 与 Nav2 的 NavigateToPose action 交互的客户端
    NavigateClient::SharedPtr m_ActionClient;

    // m_GetStateClient: 询问某个 Nav2 lifecycle 节点是否 active（默认 bt_navigator）
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr m_GetStateClient;

    // m_ManageNodesClient: 通过 lifecycle manager 启动/配置/激活 Nav2（可选）
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr m_ManageNodesClient;

    // m_AmclPoseSub: 订阅 /amcl_pose，判断定位是否已就绪
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr m_AmclPoseSub;
    std::atomic<bool> have_amcl_pose_{false};
    mutable std::mutex amcl_pose_mutex_;
    PoseWithCovarianceStamped last_amcl_pose_;

    // 参数化配置：坐标系与初始/目标位姿
    std::string map_frame_;
    std::string base_frame_;
    double initial_x_{0.0};
    double initial_y_{0.0};
    double initial_yaw_{0.0};
    double goal_x_{1.0};
    double goal_y_{0.0};
    double goal_yaw_{0.0};
    double tf_wait_timeout_sec_{10.0};
    double clock_wait_timeout_sec_{20.0};
    double nav2_active_timeout_sec_{20.0};
    double amcl_pose_timeout_sec_{10.0};
    bool use_sim_time_{false};
    bool wait_nav2_active_{true};
    bool wait_amcl_pose_{true};
    bool auto_startup_nav2_{true};

    std::string lifecycle_get_state_service_;
    std::string lifecycle_manage_nodes_service_{"/lifecycle_manager_navigation/manage_nodes"};
    std::string amcl_pose_topic_;
};
