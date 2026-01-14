// 任务编排节点实现：加载配置，依次导航到采集点并执行抓取/放置
//
// 流程图（主流程）：
//   [启动] -> [等 /clock (可选)]
//       -> [读 task_config.yaml]
//       -> [等 Nav2 Action + Pick/Place Service]
//       -> for 每个 pickup:
//            [导航到 pickup]
//            -> [调用 pick]
//            -> [导航到 dropoff]
//            -> [调用 place]
//       -> [结束]
#include "task_runner.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <exception>

using namespace std::chrono_literals;

// 构造函数：声明参数、创建 Action/Service 客户端与发布器
TaskRunner::TaskRunner()
        : Node("task_runner")
{
    // 基础参数：地图坐标系、Nav2 action 名称、任务配置路径
    m_MapFrame = this->declare_parameter<std::string>("map_frame", "map");
    m_BaseFrame = this->declare_parameter<std::string>("base_frame", "base_link");
    m_Nav2ActionName = this->declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    m_TaskConfigPath = this->declare_parameter<std::string>("task_config", "");
    // 抓取/放置服务名称
    m_PickServiceName = this->declare_parameter<std::string>("pick_service", "/manipulation/pick");
    m_PlaceServiceName = this->declare_parameter<std::string>("place_service", "/manipulation/place");
    // 超时参数：等待 /clock、单次导航超时
    m_ClockWaitTimeoutSec = this->declare_parameter<double>("clock_wait_timeout_sec", 20.0);
    m_TfWaitTimeoutSec = this->declare_parameter<double>("tf_wait_timeout_sec", 10.0);
    m_NavigationTimeoutSec = this->declare_parameter<double>("navigation_timeout_sec", 120.0);
    // 初始位姿（可选）
    m_InitialX = this->declare_parameter<double>("initial_x", 0.0);
    m_InitialY = this->declare_parameter<double>("initial_y", 0.0);
    m_InitialYaw = this->declare_parameter<double>("initial_yaw", 0.0);
    m_PublishInitialPose = this->declare_parameter<bool>("publish_initial_pose", false);
    // 是否使用仿真时间（rclcpp 会提前声明 use_sim_time，避免重复声明）
    if (this->has_parameter("use_sim_time"))
    {
        this->get_parameter("use_sim_time", m_UseSimTime);
    }
    else
    {
        m_UseSimTime = this->declare_parameter<bool>("use_sim_time", true);
    }

    // Nav2 导航 Action 客户端
    m_ActionClient = rclcpp_action::create_client<NavigateToPose>(this, m_Nav2ActionName);
    // 抓取/放置 Service 客户端
    m_PickClient = this->create_client<Trigger>(m_PickServiceName);
    m_PlaceClient = this->create_client<Trigger>(m_PlaceServiceName);

    // 发布剩余距离（由 Nav2 feedback 提供）
    m_DistancePub = this->create_publisher<std_msgs::msg::Float32>("distance_remaining", 10);
    // 发布任务状态，便于监控
    m_StatePub = this->create_publisher<ros2_learning_task_runner::msg::TaskStatus>("task_status", 10);
    // 发布初始位姿到 /initialpose（可选）
    m_InitialPosePub = this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

    set_state(TaskState::kIdle, "init");
}

// 主流程：等待环境 -> 读取配置 -> 依次执行 pickup -> dropoff
void TaskRunner::run()
{
    if (m_UseSimTime)
    {
        // 仿真时间下先等 /clock，避免 now() 始终为 0
        RCLCPP_INFO(get_logger(), "use_sim_time=true, waiting for /clock...");
        if (!wait_for_time())
        {
            set_state(TaskState::kFailed, "wait_for_time", "/clock timeout");
            RCLCPP_ERROR(get_logger(), "/clock not received within %.1f seconds.", m_ClockWaitTimeoutSec);
            return;
        }
    }

    // 读取 YAML 配置，解析 dropoff 与 pickups
    if (!load_task_config(m_TaskConfigPath))
    {
        set_state(TaskState::kFailed, "load_task_config", "invalid task config");
        RCLCPP_ERROR(get_logger(), "Failed to load task config: '%s'", m_TaskConfigPath.c_str());
        return;
    }
    m_PickupTotal = static_cast<uint32_t>(m_PickupPoints.size());

    // 等待 Nav2 action server
    if (!wait_for_action_server())
    {
        set_state(TaskState::kFailed, "wait_for_action_server", "nav2 action unavailable");
        RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable.");
        return;
    }

    if (m_PublishInitialPose)
    {
        publish_initial_pose();
    }

    if (!wait_for_tf())
    {
        set_state(TaskState::kFailed, "wait_for_tf", "tf unavailable");
        RCLCPP_ERROR(get_logger(), "TF %s -> %s unavailable.", m_MapFrame.c_str(), m_BaseFrame.c_str());
        return;
    }

    // 等待抓取/放置服务
    if (!wait_for_service(m_PickClient, m_PickServiceName))
    {
        set_state(TaskState::kFailed, "wait_for_pick_service", "pick service unavailable");
        RCLCPP_ERROR(get_logger(), "Pick service unavailable: %s", m_PickServiceName.c_str());
        return;
    }

    if (!wait_for_service(m_PlaceClient, m_PlaceServiceName))
    {
        set_state(TaskState::kFailed, "wait_for_place_service", "place service unavailable");
        RCLCPP_ERROR(get_logger(), "Place service unavailable: %s", m_PlaceServiceName.c_str());
        return;
    }

    if (!m_HasDropoff || m_PickupPoints.empty())
    {
        set_state(TaskState::kFailed, "validate_task_config", "missing dropoff or pickups");
        RCLCPP_ERROR(get_logger(), "Task config missing dropoff or pickups.");
        return;
    }

    // 主循环：每个采集点执行“去采集点 -> 抓取 -> 回投放点 -> 放置”
    for (size_t i = 0; rclcpp::ok() && i < m_PickupPoints.size(); ++i)
    {
        const auto &pickup = m_PickupPoints[i];
        m_CurrentPickupIndex = static_cast<uint32_t>(i + 1);
        RCLCPP_INFO(get_logger(), "Pickup %zu/%zu: going to (%.2f, %.2f, %.2f)",
                    i + 1, m_PickupPoints.size(), pickup.x, pickup.y, pickup.yaw);

        // 1) 导航到采集点
        set_state(TaskState::kGoingToPickup, "navigate_pickup");
        if (!navigate_to(pickup, "pickup"))
        {
            set_state(TaskState::kFailed, "navigate_pickup", "navigation to pickup failed");
            RCLCPP_WARN(get_logger(), "Navigation to pickup %zu failed, skipping.", i + 1);
            continue;
        }

        // 2) 触发抓取服务
        set_state(TaskState::kPicking, "pick");
        if (!call_trigger(m_PickClient, "pick"))
        {
            set_state(TaskState::kFailed, "pick", "pick failed");
            RCLCPP_WARN(get_logger(), "Pick failed at pickup %zu, skipping dropoff.", i + 1);
            continue;
        }

        RCLCPP_INFO(get_logger(), "Going to dropoff (%.2f, %.2f, %.2f)",
                    m_Dropoff.x, m_Dropoff.y, m_Dropoff.yaw);
        // 3) 导航到投放点
        set_state(TaskState::kGoingToDropoff, "navigate_dropoff");
        if (!navigate_to(m_Dropoff, "dropoff"))
        {
            set_state(TaskState::kFailed, "navigate_dropoff", "navigation to dropoff failed");
            RCLCPP_WARN(get_logger(), "Navigation to dropoff failed, skipping place.");
            continue;
        }

        // 4) 触发放置服务
        set_state(TaskState::kPlacing, "place");
        if (!call_trigger(m_PlaceClient, "place"))
        {
            set_state(TaskState::kFailed, "place", "place failed");
            RCLCPP_WARN(get_logger(), "Place failed after pickup %zu.", i + 1);
            continue;
        }
    }

    m_CurrentPickupIndex = 0;
    set_state(TaskState::kIdle, "completed");
    RCLCPP_INFO(get_logger(), "Task runner completed.");
}

void TaskRunner::set_state(TaskState state, const std::string &phase, const std::string &error)
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

bool TaskRunner::load_task_config(const std::string &path)
{
    if (path.empty())
    {
        RCLCPP_ERROR(get_logger(), "Parameter 'task_config' is empty.");
        return false;
    }

    // 读取 YAML 文件
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

    // 允许在 YAML 中覆盖 map_frame
    if (root["map_frame"])
    {
        m_MapFrame = root["map_frame"].as<std::string>();
    }

    // 解析投放点
    if (!parse_pose_node(root["dropoff"], "dropoff", &m_Dropoff))
    {
        return false;
    }
    m_HasDropoff = true;

    // 解析采集点列表
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
        {
            RCLCPP_ERROR(get_logger(), "Invalid pickup at index %zu.", i);
            return false;
        }
        m_PickupPoints.push_back(pose);
    }

    return true;
}

bool TaskRunner::parse_pose_node(const YAML::Node &node, const std::string &label, Pose2D *out_pose)
{
    if (!node || !out_pose)
    {
        RCLCPP_ERROR(get_logger(), "Missing %s pose in task config.", label.c_str());
        return false;
    }

    // x/y 必填，yaw 可选
    if (!node["x"] || !node["y"])
    {
        RCLCPP_ERROR(get_logger(), "%s pose requires 'x' and 'y'.", label.c_str());
        return false;
    }

    out_pose->x = node["x"].as<double>();
    out_pose->y = node["y"].as<double>();
    out_pose->yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
    return true;
}

bool TaskRunner::wait_for_time()
{
    // /clock 未到来时，now() 会是 0；这里用 wall time 等待
    const auto start_wall = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        const auto now_ns = this->get_clock()->now().nanoseconds();
        if (now_ns != 0)
        {
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() > m_ClockWaitTimeoutSec)
        {
            return false;
        }

        rclcpp::sleep_for(100ms);
    }

    return false;
}

bool TaskRunner::wait_for_tf()
{
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    RCLCPP_INFO(get_logger(), "Waiting for TF %s -> %s...", m_MapFrame.c_str(), m_BaseFrame.c_str());
    const auto start_time = this->now();
    while (rclcpp::ok())
    {
        try
        {
            const auto transform_stamped = tf_buffer.lookupTransform(m_MapFrame, m_BaseFrame, tf2::TimePointZero);
            (void)transform_stamped;
            RCLCPP_INFO(get_logger(), "TF %s -> %s is available.", m_MapFrame.c_str(), m_BaseFrame.c_str());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
            rclcpp::sleep_for(200ms);
        }

        if ((this->now() - start_time).seconds() > m_TfWaitTimeoutSec)
        {
            RCLCPP_ERROR(get_logger(), "TF wait timeout after %.1f seconds.", m_TfWaitTimeoutSec);
            return false;
        }
    }

    return false;
}

bool TaskRunner::wait_for_action_server()
{
    // 等待 Nav2 action server 可用
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server '%s'...", m_Nav2ActionName.c_str());
    return m_ActionClient->wait_for_action_server(10s);
}

bool TaskRunner::wait_for_service(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name)
{
    // 等待服务可用
    RCLCPP_INFO(get_logger(), "Waiting for service '%s'...", name.c_str());
    return client->wait_for_service(10s);
}

TaskRunner::PoseStamped TaskRunner::make_pose(const Pose2D &pose) const
{
    PoseStamped msg;
    // 坐标系与时间戳
    msg.header.frame_id = m_MapFrame;
    msg.header.stamp = this->now();
    // 位置
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;

    // yaw -> 四元数
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    return msg;
}

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

bool TaskRunner::navigate_to(const Pose2D &pose, const std::string &label)
{
    // 组装导航目标
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_pose(pose);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
        [this, label](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // 发布剩余距离，便于 UI/调试
        std_msgs::msg::Float32 msg;
        msg.data = feedback->distance_remaining;
        m_DistancePub->publish(msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[%s] remaining distance: %.2f", label.c_str(), feedback->distance_remaining);
    };

    // 发送导航目标
    auto goal_future = m_ActionClient->async_send_goal(goal_msg, send_goal_options);
    const auto send_code = rclcpp::spin_until_future_complete(shared_from_this(), goal_future, 5s);
    if (send_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Send goal failed for %s.", label.c_str());
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal rejected for %s.", label.c_str());
        return false;
    }

    // 等待导航结果（带超时）
    auto result_future = m_ActionClient->async_get_result(goal_handle);
    const std::chrono::duration<double> timeout(m_NavigationTimeoutSec);
    const auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), result_future, timeout);

    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto wrapped_result = result_future.get();
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Navigation to %s succeeded.", label.c_str());
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Navigation to %s aborted.", label.c_str());
            return false;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Navigation to %s canceled.", label.c_str());
            return false;
        default:
            RCLCPP_WARN(get_logger(), "Navigation to %s returned unknown result.", label.c_str());
            return false;
        }
    }

    // 超时则尝试取消目标
    RCLCPP_WARN(get_logger(), "Navigation to %s timed out, canceling goal.", label.c_str());
    auto cancel_future = m_ActionClient->async_cancel_goal(goal_handle);
    (void)rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
    return false;
}

bool TaskRunner::call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name)
{
    // Trigger 服务不需要请求参数
    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);
    const auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), future, 10s);
    if (result_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s service call failed.", name.c_str());
        return false;
    }

    const auto response = future.get();
    if (!response->success)
    {
        RCLCPP_WARN(get_logger(), "%s service returned failure: %s", name.c_str(), response->message.c_str());
    }

    return response->success;
}
