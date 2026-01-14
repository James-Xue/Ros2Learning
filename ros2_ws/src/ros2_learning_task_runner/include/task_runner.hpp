#pragma once

// 任务编排节点定义：读取配置并串行执行导航与抓取/放置
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_learning_task_runner/msg/task_status.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

class TaskRunner : public rclcpp::Node
{
  public:
    // 常用类型别名
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using Trigger = std_srvs::srv::Trigger;

    // 构造与主入口
    TaskRunner();

    void run();

  private:
    enum class TaskState
    {
        kIdle,
        kGoingToPickup,
        kPicking,
        kGoingToDropoff,
        kPlacing,
        kFailed
    };

    // 简化的 2D 位姿结构
    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    // 配置加载与解析
    bool load_task_config(const std::string &path);
    bool parse_pose_node(const YAML::Node &node, const std::string &label, Pose2D *out_pose);

    // 启动等待
    bool wait_for_time();
    bool wait_for_tf();
    bool wait_for_action_server();
    bool wait_for_service(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name);
    void publish_initial_pose();

    // 动作封装
    PoseStamped make_pose(const Pose2D &pose) const;
    bool navigate_to(const Pose2D &pose, const std::string &label);
    bool call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name);

    void set_state(TaskState state, const std::string &phase, const std::string &error = "");
    std::string state_to_string(TaskState state) const;

    // 参数与配置
    std::string m_MapFrame;
    std::string m_BaseFrame;
    std::string m_Nav2ActionName;
    std::string m_TaskConfigPath;
    std::string m_PickServiceName;
    std::string m_PlaceServiceName;
    double m_ClockWaitTimeoutSec{20.0};
    double m_TfWaitTimeoutSec{10.0};
    double m_NavigationTimeoutSec{120.0};
    double m_InitialX{0.0};
    double m_InitialY{0.0};
    double m_InitialYaw{0.0};
    bool m_PublishInitialPose{false};
    bool m_UseSimTime{true};

    // 任务点数据
    Pose2D m_Dropoff;
    bool m_HasDropoff{false};
    std::vector<Pose2D> m_PickupPoints;

    // ROS 通信句柄
    rclcpp_action::Client<NavigateToPose>::SharedPtr m_ActionClient;
    rclcpp::Client<Trigger>::SharedPtr m_PickClient;
    rclcpp::Client<Trigger>::SharedPtr m_PlaceClient;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_DistancePub;
    rclcpp::Publisher<ros2_learning_task_runner::msg::TaskStatus>::SharedPtr m_StatePub;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr m_InitialPosePub;

    TaskState m_CurrentState{TaskState::kIdle};
    std::string m_CurrentPhase;
    std::string m_LastError;
    uint32_t m_CurrentPickupIndex{0};
    uint32_t m_PickupTotal{0};
};
