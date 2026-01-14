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

    // 判断 lifecycle 节点是否处于 ACTIVE；机器人业务上表示导航行为树已可接管移动任务
    bool is_lifecycle_active(uint8_t state_id) const;
    // 判断 lifecycle 节点是否处于 INACTIVE；此时节点已配置但未激活，导航不可用
    bool is_lifecycle_inactive(uint8_t state_id) const;
    // 节流打印 Nav2 当前状态，避免等待阶段刷屏；便于排查导航未激活原因
    void log_nav2_state_throttled(uint8_t state_id, const std::string &state_label,
                                  std::chrono::steady_clock::time_point &last_log) const;
    // 根据 get_state 服务名推导 change_state 服务名，适配不同 Nav2 节点命名
    std::string get_change_state_service_from_get_state_service() const;
    // 通过 lifecycle manager 触发 Nav2 栈 STARTUP，适用于统一启动导航组件
    bool try_lifecycle_manager_startup();
    // 直接向具体 lifecycle 节点发送 ACTIVATE，用于 manager 不可用时的兜底
    bool try_activate_lifecycle_node();
    // 业务级“尽力而为”启动：先尝试 manager STARTUP，失败再直接 ACTIVATE
    void ensure_nav2_active_best_effort();
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
    // have_amcl_pose_: 标记是否收到过 AMCL 位姿，用于判断定位是否完成初始化
    std::atomic<bool> have_amcl_pose_;
    // amcl_pose_mutex_: 保护 last_amcl_pose_ 的并发访问，避免回调与主线程竞争
    mutable std::mutex amcl_pose_mutex_;
    // last_amcl_pose_: 最近一次 AMCL 位姿，用于输出定位状态和调试
    PoseWithCovarianceStamped last_amcl_pose_;

    // 参数化配置：坐标系与初始/目标位姿
    std::string map_frame_;
    std::string base_frame_;
    double initial_x_;
    double initial_y_;
    double initial_yaw_;
    double goal_x_;
    double goal_y_;
    double goal_yaw_;
    double tf_wait_timeout_sec_;
    double clock_wait_timeout_sec_;
    double nav2_active_timeout_sec_;
    double amcl_pose_timeout_sec_;
    bool use_sim_time_;
    bool wait_nav2_active_;
    bool wait_amcl_pose_;
    bool auto_startup_nav2_;

    std::string lifecycle_get_state_service_;
    std::string lifecycle_manage_nodes_service_;
    std::string amcl_pose_topic_;
};
