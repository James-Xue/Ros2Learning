#pragma once

// ==============================================================================
// 任务编排节点 (TaskRunner) 头文件
//
// 对于 ROS 2 新手的说明：
// 1. ROS 2 中的“节点”(Node) 是执行具体任务的最小进程单元。
// 2. 本类继承自 rclcpp::Node，使其具备了发布话题(Publisher)、订阅服务(Client)
//    以及执行动作(ActionClient) 的能力。
// ==============================================================================

#include <string>
#include <vector>

// --- ROS 2 核心头文件 ---
// rclcpp 是 ROS 2 的 C++ 客户端库，提供 Node, Publisher, Client 等核心功能
#include "rclcpp/rclcpp.hpp"
// 提供 Action (动作) 客户端与服务端的支持
#include "rclcpp_action/rclcpp_action.hpp"

// --- 消息格式 (Message) 头文件 ---
// ROS 2 中的数据通过“消息”传递。每个 .msg 文件都会生成对应的 .hpp
// 带坐标系信息的位姿 (x,y,z + orientation)
#include "geometry_msgs/msg/pose_stamped.hpp"
// 带协方差的位姿（常用于初始位姿 /initialpose）
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// 自定义的消息类型，用于汇报本节点的任务状态
#include "ros2_learning_task_runner/msg/task_status.hpp"
// 标准浮点数消息，用于发布距离
#include "std_msgs/msg/float32.hpp"

// --- 服务 (Service) 与 动作 (Action) 头文件 ---
// Nav2 提供的“导航到目标点”动作
#include "nav2_msgs/action/navigate_to_pose.hpp"
// 标准触发服务 (Trigger)，只有 Request 和 Response，常用于“开始/停止”指令
#include "std_srvs/srv/trigger.hpp"

// --- 外部库 ---
#include "yaml-cpp/yaml.h" // 用于解析 .yaml 配置文件

class TaskRunner : public rclcpp::Node
{
  public:
    // ==========================================================================
    // 类型别名 (Using Aliases)
    // 主要是为了简化代码中过长的模板类名。
    // ==========================================================================
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle =
        rclcpp_action::ClientGoalHandle<NavigateToPose>; // 管理 Action
                                                         // 的目标句柄
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped =
        geometry_msgs::msg::PoseWithCovarianceStamped;
    using Trigger = std_srvs::srv::Trigger;

    /**
     * @brief 构造函数：初始化 ROS 2 节点标识符、加载参数和创建通信接口
     */
    TaskRunner();

    /**
     * @brief 任务主运行逻辑：负责串行化执行整个工作流
     */
    void run();

  private:
    /**
     * @brief 内部任务状态枚举
     */
    enum class TaskState
    {
        kIdle,           // 空闲状态
        kGoingToPickup,  // 正在前往采集点
        kPicking,        // 正在执行抓取动作
        kGoingToDropoff, // 正在前往投放点
        kPlacing,        // 正在执行放置动作
        kFailed          // 任务失败
    };

    /**
     * @brief 简单的 2D 位姿结构，方便从 YAML 加载
     */
    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0}; // 偏航角，弧度制
    };

    // --- 配置加载函数 ---
    bool load_task_config(const std::string &path);
    bool parse_pose_node(const YAML::Node &node, const std::string &label,
                         Pose2D *out_pose);

    // --- 环境准备相关函数 ---
    bool wait_for_time();          // 等待仿真时间 (/clock) 同步
    bool wait_for_tf();            // 等待坐标系变换 (TF) 可用
    bool wait_for_action_server(); // 等待导航服务启动
    bool wait_for_service(const rclcpp::Client<Trigger>::SharedPtr &client,
                          const std::string &name);
    void publish_initial_pose(); // 发布初始位置到 AMCL

    // --- 业务操作函数 ---
    PoseStamped make_pose(
        const Pose2D &pose) const; // 将 2D 位姿转为 ROS 标准的 PoseStamped
    bool navigate_to(const Pose2D &pose,
                     const std::string &label); // 调用 Nav2 执行导航
    bool call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client,
                      const std::string &name); // 调用抓取/放置服务

    // --- 内部辅助 ---
    void set_state(TaskState state, const std::string &phase,
                   const std::string &error = "");
    std::string state_to_string(TaskState state) const;

    // ==========================================================================
    // 成员变量
    // ==========================================================================

    // --- 配置与参数 ---
    std::string m_MapFrame;  // 地图坐标系名称 (通常是 "map")
    std::string m_BaseFrame; // 机器人坐标系名称 (通常是 "base_link")
    std::string m_Nav2ActionName;   // Nav2 动作服务的名称
    std::string m_TaskConfigPath;   // 任务配置 YAML 文件的绝对路径
    std::string m_PickServiceName;  // 机械臂抓取服务的名称
    std::string m_PlaceServiceName; // 机械臂放置服务的名称

    // --- 超时与控制 ---
    double m_ClockWaitTimeoutSec{20.0};
    double m_TfWaitTimeoutSec{10.0};
    double m_NavigationTimeoutSec{120.0};
    double m_InitialX{0.0};
    double m_InitialY{0.0};
    double m_InitialYaw{0.0};
    bool m_PublishInitialPose{false}; // 是否自动发布初始位姿
    bool m_UseSimTime{true};          // 是否使用仿真时钟

    // --- 任务轨迹点 ---
    Pose2D m_Dropoff; // 固定的投放点位姿
    bool m_HasDropoff{false};
    std::vector<Pose2D> m_PickupPoints; // 多个采集点的列表

    // ==========================================================================
    // ROS 2 通信接口句柄 (Shared Pointers)
    // ==========================================================================

    // Action Client: 负责异步导航任务，可以获取反馈（如距离）和结果
    rclcpp_action::Client<NavigateToPose>::SharedPtr m_ActionClient;

    // Service Clients: 负责同步/简单的服务调用（抓取、放置）
    rclcpp::Client<Trigger>::SharedPtr m_PickClient;
    rclcpp::Client<Trigger>::SharedPtr m_PlaceClient;

    // Publishers: 负责将信息广播出去，供其他节点（如 UI、调试工具）查看
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        m_DistancePub; // 发布剩余距离
    rclcpp::Publisher<ros2_learning_task_runner::msg::TaskStatus>::SharedPtr
        m_StatePub; // 发布核心任务状态
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr
        m_InitialPosePub; // 发布 AMCL 初始化位姿

    // --- 运行状态 ---
    TaskState m_CurrentState{TaskState::kIdle};
    std::string m_CurrentPhase;
    std::string m_LastError;
    uint32_t m_CurrentPickupIndex{0};
    uint32_t m_PickupTotal{0};
};
