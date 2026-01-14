// ==============================================================================
// 任务编排节点 (TaskRunner) 实现文件
//
// 本文件实现了在头文件中声明的所有功能，包括：
// 1. 从 ROS 2 参数服务器读取配置。
// 2. 依次与 Nav2 (导航) 和 机械臂 (抓取/放置) 协作。
// ==============================================================================

#include "task_runner.hpp"

// --- TF2 坐标变换相关 ---
// 四元数工具，用于处理机器人的朝向
#include <tf2/LinearMath/Quaternion.h>
// 提供 Pose 和 TF 之间的转换函数
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 存储坐标变换数据的缓冲区
#include <tf2_ros/buffer.h>
// 监听 TF 话题并填充缓冲区的监听器
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <exception>

// 使用 chrono 库的字面量，例如 10s, 100ms，增加代码可读性
using namespace std::chrono_literals;

/**
 * @brief 构造函数实现
 *
 * ROS 2 节点的生命周期通常从构造函数开始：
 * 1. 初始化父类 Node("节点名")。
 * 2. 声明和读取参数 (declare_parameter)。
 * 3. 创建通信接口 (Publisher, Client, ActionClient)。
 */
TaskRunner::TaskRunner()
        : Node("task_runner")
{
    // --- 声明与加载参数 ---
    // declare_parameter 会尝试从命令行、launch 文件或 yaml
    // 获取值，若没有则使用默认值
    m_MapFrame = this->declare_parameter<std::string>("map_frame", "map");
    m_BaseFrame =
        this->declare_parameter<std::string>("base_frame", "base_link");
    m_Nav2ActionName = this->declare_parameter<std::string>("nav2_action_name",
                                                            "navigate_to_pose");
    m_TaskConfigPath = this->declare_parameter<std::string>("task_config", "");

    m_PickServiceName = this->declare_parameter<std::string>(
        "pick_service", "/manipulation/pick");
    m_PlaceServiceName = this->declare_parameter<std::string>(
        "place_service", "/manipulation/place");

    m_ClockWaitTimeoutSec =
        this->declare_parameter<double>("clock_wait_timeout_sec", 20.0);
    m_TfWaitTimeoutSec =
        this->declare_parameter<double>("tf_wait_timeout_sec", 10.0);
    m_NavigationTimeoutSec =
        this->declare_parameter<double>("navigation_timeout_sec", 120.0);

    m_InitialX = this->declare_parameter<double>("initial_x", 0.0);
    m_InitialY = this->declare_parameter<double>("initial_y", 0.0);
    m_InitialYaw = this->declare_parameter<double>("initial_yaw", 0.0);
    m_PublishInitialPose =
        this->declare_parameter<bool>("publish_initial_pose", false);

    // 仿真时间处理：如果外部 launch 文件设置了 use_sim_time，Node 会自动同步
    if (this->has_parameter("use_sim_time"))
    {
        this->get_parameter("use_sim_time", m_UseSimTime);
    }
    else
    {
        m_UseSimTime = this->declare_parameter<bool>("use_sim_time", true);
    }

    // --- 创建通信接口 ---

    // 1. Action Client (动作客户端)
    // 用于调用 Nav2。动作包含：目标(Goal)、反馈(Feedback)、结果(Result)
    m_ActionClient =
        rclcpp_action::create_client<NavigateToPose>(this, m_Nav2ActionName);

    // 2. Service Client (服务客户端)
    // 用于调用抓取和放置。属于“一问一答”模式，适合简单的命令触发
    m_PickClient = this->create_client<Trigger>(m_PickServiceName);
    m_PlaceClient = this->create_client<Trigger>(m_PlaceServiceName);

    // 3. Publisher (发布器)
    // 发布话题数据，遵循“观察者模式”。
    // 队列大小 (History Depth) 设为 10，防止瞬时并发消息丢失
    m_DistancePub = this->create_publisher<std_msgs::msg::Float32>(
        "distance_remaining", 10);
    m_StatePub =
        this->create_publisher<ros2_learning_task_runner::msg::TaskStatus>(
            "task_status", 10);
    m_InitialPosePub =
        this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

    set_state(TaskState::kIdle, "init");
}

/**
 * @brief 执行任务逻辑
 *
 * 这是一个串行工作流示例。在复杂的机器人系统中，通常会使用“行为树”(Behavior
 * Tree) 来管理这种逻辑，但对于新手学习，线性逻辑是最直观的。
 */
void TaskRunner::run()
{
    // 如果是仿真环境，必须先等到有时钟产生
    if (m_UseSimTime)
    {
        RCLCPP_INFO(get_logger(), "use_sim_time=true, waiting for /clock...");
        if (!wait_for_time())
        {
            set_state(TaskState::kFailed, "wait_for_time", "/clock timeout");
            return;
        }
    }

    // 1. 加载任务点
    if (!load_task_config(m_TaskConfigPath))
    {
        set_state(TaskState::kFailed, "load_task_config",
                  "invalid task config");
        return;
    }
    m_PickupTotal = static_cast<uint32_t>(m_PickupPoints.size());

    // 2. 等待外部服务就绪
    // 在机器人启动流程中，各节点启动顺序不一，确保依赖的服务已在线是非常关键的。
    if (!wait_for_action_server())
    {
        set_state(TaskState::kFailed, "wait_for_action_server",
                  "nav2 action unavailable");
        return;
    }

    if (m_PublishInitialPose)
    {
        publish_initial_pose();
    }

    if (!wait_for_tf())
    {
        set_state(TaskState::kFailed, "wait_for_tf", "tf unavailable");
        return;
    }

    if (!wait_for_service(m_PickClient, m_PickServiceName) ||
        !wait_for_service(m_PlaceClient, m_PlaceServiceName))
    {
        set_state(TaskState::kFailed, "wait_for_services",
                  "manipulation services unavailable");
        return;
    }

    if (!m_HasDropoff || m_PickupPoints.empty())
    {
        set_state(TaskState::kFailed, "validate_task_config",
                  "missing dropoff or pickups");
        return;
    }

    // 3. 执行循环任务
    for (size_t i = 0; rclcpp::ok() && i < m_PickupPoints.size(); ++i)
    {
        const auto &pickup = m_PickupPoints[i];
        m_CurrentPickupIndex = static_cast<uint32_t>(i + 1);

        // --- 阶段 A: 前往采集点 ---
        set_state(TaskState::kGoingToPickup, "navigate_pickup");
        if (!navigate_to(pickup, "pickup"))
        {
            set_state(TaskState::kFailed, "navigate_pickup",
                      "navigation to pickup failed");
            continue;
        }

        // --- 阶段 B: 执行抓取 ---
        set_state(TaskState::kPicking, "pick");
        if (!call_trigger(m_PickClient, "pick"))
        {
            set_state(TaskState::kFailed, "pick", "pick failed");
            continue;
        }

        // --- 阶段 C: 前往投放点 ---
        set_state(TaskState::kGoingToDropoff, "navigate_dropoff");
        if (!navigate_to(m_Dropoff, "dropoff"))
        {
            set_state(TaskState::kFailed, "navigate_dropoff",
                      "navigation to dropoff failed");
            continue;
        }

        // --- 阶段 D: 执行放置 ---
        set_state(TaskState::kPlacing, "place");
        if (!call_trigger(m_PlaceClient, "place"))
        {
            set_state(TaskState::kFailed, "place", "place failed");
            continue;
        }
    }

    m_CurrentPickupIndex = 0;
    set_state(TaskState::kIdle, "completed");
    RCLCPP_INFO(get_logger(), "Task runner completed.");
}

/**
 * @brief 更新并发布节点状态
 *
 * 这是一个良好的工程实践：将内部变量变化封装在函数中，并自动发布状态消息以便调试和
 * UI 展示。
 */
void TaskRunner::set_state(TaskState state, const std::string &phase,
                           const std::string &error)
{
    m_CurrentState = state;
    m_CurrentPhase = phase;
    if (!error.empty())
    {
        m_LastError = error;
    }
    else if (state != TaskState::kFailed)
    {
        m_LastError.clear();
    }

    ros2_learning_task_runner::msg::TaskStatus msg;
    msg.state = state_to_string(state);
    msg.phase = m_CurrentPhase;
    msg.pickup_index = m_CurrentPickupIndex;
    msg.pickup_total = m_PickupTotal;
    msg.last_error = m_LastError;
    m_StatePub->publish(msg);
}

std::string TaskRunner::state_to_string(TaskState state) const
{
    switch (state)
    {
    case TaskState::kIdle:
        return "idle";
    case TaskState::kGoingToPickup:
        return "going_to_pickup";
    case TaskState::kPicking:
        return "picking";
    case TaskState::kGoingToDropoff:
        return "going_to_dropoff";
    case TaskState::kPlacing:
        return "placing";
    case TaskState::kFailed:
        return "failed";
    default:
        return "unknown";
    }
}

/**
 * @brief 解析 YAML 配置文件 (使用 yaml-cpp)
 */
bool TaskRunner::load_task_config(const std::string &path)
{
    if (path.empty())
    {
        RCLCPP_ERROR(get_logger(), "Parameter 'task_config' is empty.");
        return false;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "YAML load error: %s", e.what());
        return false;
    }

    if (root["map_frame"])
    {
        m_MapFrame = root["map_frame"].as<std::string>();
    }

    // 解析 dropoff
    if (!parse_pose_node(root["dropoff"], "dropoff", &m_Dropoff))
        return false;
    m_HasDropoff = true;

    // 解析 pickups 列表
    m_PickupPoints.clear();
    const auto pickups = root["pickups"];
    if (!pickups || !pickups.IsSequence())
    {
        RCLCPP_ERROR(get_logger(), "'pickups' must be a list in task config.");
        return false;
    }

    for (size_t i = 0; i < pickups.size(); ++i)
    {
        Pose2D pose;
        if (!parse_pose_node(pickups[i], "pickup", &pose))
            return false;
        m_PickupPoints.push_back(pose);
    }

    return true;
}

bool TaskRunner::parse_pose_node(const YAML::Node &node,
                                 const std::string &label, Pose2D *out_pose)
{
    if (!node || !out_pose)
        return false;
    if (!node["x"] || !node["y"])
    {
        RCLCPP_ERROR(get_logger(), "%s pose requires 'x' and 'y'.",
                     label.c_str());
        return false;
    }

    out_pose->x = node["x"].as<double>();
    out_pose->y = node["y"].as<double>();
    out_pose->yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
    return true;
}

/**
 * @brief 环境同步：等待仿真时间产生
 */
bool TaskRunner::wait_for_time()
{
    const auto start_wall = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        // 尝试获取当前时间。在仿真或真实硬件就绪前，初值为 0
        if (this->get_clock()->now().nanoseconds() != 0)
            return true;

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > m_ClockWaitTimeoutSec)
        {
            return false;
        }
        rclcpp::sleep_for(100ms);
    }
    return false;
}

/**
 * @brief 坐标变换等待：确保 map -> base_link 链路通畅
 */
bool TaskRunner::wait_for_tf()
{
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    RCLCPP_INFO(get_logger(), "Waiting for TF %s -> %s...", m_MapFrame.c_str(),
                m_BaseFrame.c_str());
    const auto start_time = this->now();
    while (rclcpp::ok())
    {
        try
        {
            // lookupTransform 用于查询两个坐标系之间的变换关系
            tf_buffer.lookupTransform(m_MapFrame, m_BaseFrame,
                                      tf2::TimePointZero);
            RCLCPP_INFO(get_logger(), "TF %s -> %s is available.",
                        m_MapFrame.c_str(), m_BaseFrame.c_str());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            rclcpp::sleep_for(200ms);
        }

        if ((this->now() - start_time).seconds() > m_TfWaitTimeoutSec)
            return false;
    }
    return false;
}

bool TaskRunner::wait_for_action_server()
{
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server '%s'...",
                m_Nav2ActionName.c_str());
    return m_ActionClient->wait_for_action_server(10s);
}

bool TaskRunner::wait_for_service(
    const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name)
{
    RCLCPP_INFO(get_logger(), "Waiting for service '%s'...", name.c_str());
    return client->wait_for_service(10s);
}

/**
 * @brief 构造 ROS 标准位姿消息 (PoseStamped)
 */
TaskRunner::PoseStamped TaskRunner::make_pose(const Pose2D &pose) const
{
    PoseStamped msg;
    // Header 包含坐标系标识和时间戳，是 ROS 2 消息空间位置感知的核心
    msg.header.frame_id = m_MapFrame;
    msg.header.stamp = this->now();

    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;

    // ROS 内部使用四元数 (Quaternion) 表示 3D 旋转。
    // 我们需要通过 tf2 工具库将简单的 2D yaw 角转换为四元数。
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    return msg;
}

/**
 * @brief 发布初始位姿 (用于 AMCL 等定位算法的初始化)
 */
void TaskRunner::publish_initial_pose()
{
    PoseWithCovarianceStamped msg;
    msg.header.frame_id = m_MapFrame;
    msg.header.stamp = this->now();
    msg.pose.pose.position.x = m_InitialX;
    msg.pose.pose.position.y = m_InitialY;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, m_InitialYaw);
    msg.pose.pose.orientation = tf2::toMsg(q);

    // 协方差矩阵：这里设置 0.25 (即标准差 0.5m) 的对角阵
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.25;

    RCLCPP_INFO(get_logger(), "Publishing initial pose to /initialpose");
    for (int i = 0; rclcpp::ok() && i < 5; ++i)
    {
        msg.header.stamp = this->now();
        m_InitialPosePub->publish(msg);
        rclcpp::sleep_for(200ms);
    }
}

/**
 * @brief 调用 Nav2 导航动作 (异步模式 + 持久化反馈处理)
 *
 * 对于新手，这里涉及到 ROS 2 C++ 最复杂的几个概念：
 * 1. async_send_goal：发送请求后程序不阻塞，继续往下走。
 * 2. Lambda 表达式用于处理反馈回调 (feedback_callback)。
 * 3. spin_until_future_complete：手动推进事件循环直到动作完成。
 */
bool TaskRunner::navigate_to(const Pose2D &pose, const std::string &label)
{
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_pose(pose);

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // 反馈回调：在导航过程中，Nav2 会不断发回数据（如剩余距离）
    send_goal_options.feedback_callback =
        [this,
         label](GoalHandle::SharedPtr,
                const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        std_msgs::msg::Float32 msg;
        msg.data = feedback->distance_remaining;
        m_DistancePub->publish(msg);

        // RCLCPP_INFO_THROTTLE 控制日志打印速率，避免刷屏
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[%s] remaining distance: %.2f", label.c_str(),
                             feedback->distance_remaining);
    };

    // 1. 发送目标请求
    auto goal_future =
        m_ActionClient->async_send_goal(goal_msg, send_goal_options);

    // 等待服务器确认接收目标 (设置 5s 超时)
    const auto send_code =
        rclcpp::spin_until_future_complete(shared_from_this(), goal_future, 5s);
    if (send_code != rclcpp::FutureReturnCode::SUCCESS)
        return false;

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal rejected for %s.", label.c_str());
        return false;
    }

    // 2. 等待导航最终执行结果 (带全局超时控制)
    auto result_future = m_ActionClient->async_get_result(goal_handle);
    const std::chrono::duration<double> timeout(m_NavigationTimeoutSec);
    const auto result_code = rclcpp::spin_until_future_complete(
        shared_from_this(), result_future, timeout);

    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto wrapped_result = result_future.get();
        return (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }

    // 如果超时，必须取消当前目标，防止机器人继续非法运行
    RCLCPP_WARN(get_logger(), "Navigation to %s timed out, canceling goal.",
                label.c_str());
    auto cancel_future = m_ActionClient->async_cancel_goal(goal_handle);
    (void)rclcpp::spin_until_future_complete(shared_from_this(), cancel_future,
                                             5s);
    return false;
}

/**
 * @brief 通用 Service 调用封装
 */
bool TaskRunner::call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client,
                              const std::string &name)
{
    // 构造请求数据 (Trigger 无需额外参数)
    auto request = std::make_shared<Trigger::Request>();

    // 异步发送请求并获取 future
    auto future = client->async_send_request(request);

    // 等待响应数据
    const auto result_code =
        rclcpp::spin_until_future_complete(shared_from_this(), future, 10s);
    if (result_code != rclcpp::FutureReturnCode::SUCCESS)
        return false;

    const auto response = future.get();
    if (!response->success)
    {
        RCLCPP_WARN(get_logger(), "%s failed: %s", name.c_str(),
                    response->message.c_str());
    }
    return response->success;
}
