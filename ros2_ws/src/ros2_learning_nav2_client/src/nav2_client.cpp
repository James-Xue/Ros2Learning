// 包含本包对应的头文件，声明了 Nav2Client 类与相关类型
#include "nav2_client.hpp"

// 引入 tf2 相关异常类型（用于捕获 transform 查找中的异常）
#include <tf2/exceptions.h>
// 引入 TF2 的 Buffer，用来缓存和查询坐标变换
#include <tf2_ros/buffer.h>
// 引入 TF2 的 TransformListener，用于订阅并把变换写入 Buffer
#include <tf2_ros/transform_listener.h>

#include <chrono>

#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

// 使用 std::chrono 的字面量（如 100ms, 1s 等）方便写时长常量
using namespace std::chrono_literals;

// 构造函数：初始化为一个 rclcpp 节点，节点名为 "nav2_minimal_client"
Nav2Client::Nav2Client()
        : Node("nav2_minimal_client")
{
    // 参数声明与读取：允许通过 ROS 参数覆盖默认值
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);
    goal_x_ = this->declare_parameter<double>("goal_x", 1.0);
    goal_y_ = this->declare_parameter<double>("goal_y", 0.0);
    goal_yaw_ = this->declare_parameter<double>("goal_yaw", 0.0);

    tf_wait_timeout_sec_ = this->declare_parameter<double>("tf_wait_timeout_sec", 10.0);
    clock_wait_timeout_sec_ = this->declare_parameter<double>("clock_wait_timeout_sec", 20.0);

    lifecycle_get_state_service_ =
        this->declare_parameter<std::string>("lifecycle_get_state_service", "/bt_navigator/get_state");
    lifecycle_manage_nodes_service_ = this->declare_parameter<std::string>(
        "lifecycle_manage_nodes_service", "/lifecycle_manager_navigation/manage_nodes");
    nav2_active_timeout_sec_ = this->declare_parameter<double>("nav2_active_timeout_sec", 20.0);
    wait_nav2_active_ = this->declare_parameter<bool>("wait_nav2_active", true);
    auto_startup_nav2_ = this->declare_parameter<bool>("auto_startup_nav2", true);

    amcl_pose_topic_ = this->declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    amcl_pose_timeout_sec_ = this->declare_parameter<double>("amcl_pose_timeout_sec", 10.0);
    wait_amcl_pose_ = this->declare_parameter<bool>("wait_amcl_pose", true);

    // 默认 true：本仓库的 nav2_client 主要用于 Gazebo/Nav2 仿真栈，若不启用 sim time，
    // 发送 initialpose/goal 时的时间戳会是系统时间，可能导致 Nav2 侧判定异常并 ABORT。
    // 注意：rclcpp 可能会在内部自动声明 use_sim_time（TimeSource）。
    // 因此这里要避免重复 declare 导致: "parameter 'use_sim_time' has already been declared"。
    if (!this->has_parameter("use_sim_time"))
    {
        use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
    }
    else
    {
        this->get_parameter("use_sim_time", use_sim_time_);
    }

    m_GetStateClient = this->create_client<lifecycle_msgs::srv::GetState>(lifecycle_get_state_service_);
    m_ManageNodesClient = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(lifecycle_manage_nodes_service_);

    m_AmclPoseSub =
        this->create_subscription<PoseWithCovarianceStamped>(amcl_pose_topic_, 10,
                                                             [this](const PoseWithCovarianceStamped::SharedPtr msg)
                                                             {
                                                                 {
                                                                     std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
                                                                     last_amcl_pose_ = *msg;
                                                                 }
                                                                 have_amcl_pose_.store(true, std::memory_order_release);
                                                             });

    // 创建一个发布器，用于发布初始位姿到 /initialpose 话题，队列深度为 10
    m_InitialPosePublisher = this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);
    // 创建一个 action 客户端，用来与 Nav2 的 navigate_to_pose action 通信
    m_ActionClient = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

// 入口运行函数：负责等待时间、等待 action server、发布初始位姿、发送目标并等待结果
void Nav2Client::run()
{
    // 在仿真时等待 /clock（use_sim_time=true），否则跳过等待
    if (use_sim_time_)
    {
        RCLCPP_INFO(get_logger(), "use_sim_time=true, waiting for /clock...");
        const bool clock_ok = wait_for_time();
        if (!clock_ok)
        {
            RCLCPP_ERROR(get_logger(),
                         "/clock not received within %.1f seconds. "
                         "Start the simulation/Nav2 stack first, or run with -p use_sim_time:=false.",
                         clock_wait_timeout_sec_);
            return;
        }
    }

    // 等待 Nav2 的 action server 可用，如果不可用则报错并返回
    const bool is_server_ready = wait_for_server();
    if (!is_server_ready)
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable.");
        return;
    }

    // 等待 Nav2 lifecycle manager 报告 active（避免过早发 goal 被拒）
    if (wait_nav2_active_)
    {
        const bool nav2_active_ok = wait_for_nav2_active();
        if (!nav2_active_ok)
        {
            RCLCPP_WARN(get_logger(), "Nav2 stack not active within timeout; proceeding anyway.");
        }
    }

    // 发布初始位姿到 /initialpose，便于在 RViz 或 AMCL 中设置初始位姿
    publish_initial_pose();

    // 等待 AMCL 位姿（确保 initialpose 被处理；否则 BT Navigator 可能拒绝 goal）
    if (wait_amcl_pose_)
    {
        const bool amcl_ok = wait_for_amcl_pose();
        if (!amcl_ok)
        {
            RCLCPP_WARN(get_logger(), "No AMCL pose received within timeout; continuing.");
        }
    }

    // 等待 TF 中 map->base_link 的变换可用，确保里程计/定位信息就绪
    const bool is_tf_ready = wait_for_tf();
    if (!is_tf_ready)
    {
        RCLCPP_ERROR(get_logger(), "TF is not available within timeout, aborting.");
        return;
    }

    // 发送导航目标并获取 goal_handle
    auto goal_handle = send_goal();
    // 如果发送失败（返回空句柄），打印错误并返回
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send goal.");
        return;
    }

    // 异步获取结果的 future
    auto result_future = m_ActionClient->async_get_result(goal_handle);
    // 等待结果，最长等待 60 秒
    const auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), result_future, 60s);

    // 如果等待成功（future 返回），处理结果码
    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        // 获取封装的结果对象
        const auto wrapped_result = result_future.get();
        const auto action_result_code = wrapped_result.code;
        const auto result = wrapped_result.result;
        const auto error_code = result ? result->error_code : 0;
        const auto error_msg = (result && !result->error_msg.empty()) ? result->error_msg.c_str() : "";
        // 根据 action 返回的 code 进行不同日志输出
        switch (action_result_code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // 导航成功
            RCLCPP_INFO(get_logger(), "Navigation succeeded. error_code=%u error_msg='%s'", error_code, error_msg);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            // 导航被中止
            RCLCPP_WARN(get_logger(), "Navigation aborted. error_code=%u error_msg='%s'", error_code, error_msg);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            // 导航被取消
            RCLCPP_WARN(get_logger(), "Navigation canceled. error_code=%u error_msg='%s'", error_code, error_msg);
            break;
        default:
            // 未知结果码
            RCLCPP_WARN(get_logger(), "Unknown result code. code=%d error_code=%u error_msg='%s'",
                        static_cast<int>(action_result_code), error_code, error_msg);
            break;
        }
    }
    else if (result_code == rclcpp::FutureReturnCode::TIMEOUT)
    {
        // 如果等待超时或 future 未成功返回，则尝试取消目标
        RCLCPP_WARN(get_logger(), "Navigation timeout, canceling goal.");

        if (!rclcpp::ok())
        {
            RCLCPP_WARN(get_logger(), "ROS is shutting down; skip cancel request.");
            return;
        }

        auto cancel_future = m_ActionClient->async_cancel_goal(goal_handle);
        // 等待取消操作完成，最长等待 5 秒
        const auto cancel_code = rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
        (void)cancel_code;
    }
    else
    {
        // e.g. Ctrl-C: rclcpp::FutureReturnCode::INTERRUPTED
        RCLCPP_WARN(get_logger(), "Navigation wait interrupted.");
        return;
    }
}

void Nav2Client::spin_some_for(std::chrono::nanoseconds duration)
{
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(shared_from_this());

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        exec.spin_some();

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= duration)
        {
            break;
        }

        std::this_thread::sleep_for(20ms);
    }
}

bool Nav2Client::wait_for_nav2_active()
{
    if (!m_GetStateClient)
    {
        return false;
    }

    RCLCPP_INFO(get_logger(), "Waiting for Nav2 lifecycle get_state service: %s", lifecycle_get_state_service_.c_str());
    if (!m_GetStateClient->wait_for_service(5s))
    {
        RCLCPP_WARN(get_logger(), "Service not available: %s", lifecycle_get_state_service_.c_str());
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    auto last_log = start;
    bool startup_sent = false;
    while (rclcpp::ok())
    {
        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = m_GetStateClient->async_send_request(req);

        const auto rc = rclcpp::spin_until_future_complete(shared_from_this(), future, 1s);
        if (rc == rclcpp::FutureReturnCode::SUCCESS)
        {
            const auto resp = future.get();
            if (resp)
            {
                const auto now = std::chrono::steady_clock::now();
                if (now - last_log >= 1s)
                {
                    RCLCPP_INFO(get_logger(), "Nav2 state: id=%u label='%s' (%s)", resp->current_state.id,
                                resp->current_state.label.c_str(), lifecycle_get_state_service_.c_str());
                    last_log = now;
                }

                if (resp->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                {
                    RCLCPP_INFO(get_logger(), "Nav2 lifecycle node is ACTIVE (%s)",
                                lifecycle_get_state_service_.c_str());
                    return true;
                }

                // If it's inactive, optionally try to startup the stack via lifecycle manager.
                if (!startup_sent && auto_startup_nav2_ &&
                    resp->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                {
                    auto try_activate_directly = [this]() -> void
                    {
                        std::string change_state_service = "/bt_navigator/change_state";
                        const std::string suffix = "/get_state";
                        if (lifecycle_get_state_service_.size() > suffix.size() &&
                            lifecycle_get_state_service_.compare(lifecycle_get_state_service_.size() - suffix.size(),
                                                                 suffix.size(), suffix) == 0)
                        {
                            const std::string node_name = lifecycle_get_state_service_.substr(
                                0, lifecycle_get_state_service_.size() - suffix.size());
                            change_state_service = node_name + "/change_state";
                        }

                        auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_service);
                        if (!client->wait_for_service(1s))
                        {
                            RCLCPP_WARN(this->get_logger(), "Lifecycle change_state service not available: %s",
                                        change_state_service.c_str());
                            return;
                        }

                        auto req3 = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
                        req3->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
                        auto fut3 = client->async_send_request(req3);
                        const auto rc3 = rclcpp::spin_until_future_complete(this->shared_from_this(), fut3, 5s);
                        if (rc3 == rclcpp::FutureReturnCode::SUCCESS)
                        {
                            const auto resp3 = fut3.get();
                            const bool ok = resp3 && resp3->success;
                            RCLCPP_WARN(this->get_logger(), "Tried direct ACTIVATE (%s): success=%s",
                                        change_state_service.c_str(), ok ? "true" : "false");
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "Direct ACTIVATE call did not complete (%s)",
                                        change_state_service.c_str());
                        }
                    };

                    if (m_ManageNodesClient && m_ManageNodesClient->wait_for_service(1s))
                    {
                        auto req2 = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
                        req2->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
                        auto fut2 = m_ManageNodesClient->async_send_request(req2);
                        const auto rc2 = rclcpp::spin_until_future_complete(shared_from_this(), fut2, 5s);
                        if (rc2 == rclcpp::FutureReturnCode::SUCCESS)
                        {
                            const auto resp2 = fut2.get();
                            const bool ok = resp2 && resp2->success;
                            RCLCPP_WARN(get_logger(), "Sent STARTUP to lifecycle manager (%s): success=%s",
                                        lifecycle_manage_nodes_service_.c_str(), ok ? "true" : "false");
                            if (!ok)
                            {
                                try_activate_directly();
                            }
                        }
                        else
                        {
                            RCLCPP_WARN(get_logger(), "Failed to call lifecycle manager STARTUP (%s)",
                                        lifecycle_manage_nodes_service_.c_str());
                            try_activate_directly();
                        }
                        startup_sent = true;
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "Lifecycle manager service not available: %s",
                                    lifecycle_manage_nodes_service_.c_str());
                        try_activate_directly();
                        startup_sent = true;
                    }
                }
            }
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() > nav2_active_timeout_sec_)
        {
            return false;
        }

        std::this_thread::sleep_for(200ms);
    }

    return false;
}

bool Nav2Client::wait_for_amcl_pose()
{
    if (amcl_pose_topic_.empty())
    {
        return false;
    }

    if (have_amcl_pose_.load(std::memory_order_acquire))
    {
        RCLCPP_INFO(get_logger(), "AMCL pose already received on %s", amcl_pose_topic_.c_str());
        return true;
    }

    RCLCPP_INFO(get_logger(), "Waiting for AMCL pose on %s...", amcl_pose_topic_.c_str());

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        spin_some_for(200ms);
        if (have_amcl_pose_.load(std::memory_order_acquire))
        {
            std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
            RCLCPP_INFO(get_logger(), "AMCL pose received: (%.3f, %.3f)", last_amcl_pose_.pose.pose.position.x,
                        last_amcl_pose_.pose.pose.position.y);
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() > amcl_pose_timeout_sec_)
        {
            return false;
        }
    }

    return false;
}

// 等待 TF 中 map -> base_link 变换可用的函数
bool Nav2Client::wait_for_tf()
{
    // 使用节点的时钟构造一个 tf2 buffer，用来缓存变换
    tf2_ros::Buffer tf_buffer(this->get_clock());
    // 构造 TransformListener，它会订阅 /tf 或 /tf_static 并填充上面的 buffer
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 打印等待信息
    RCLCPP_INFO(get_logger(), "Waiting for TF %s -> %s...", map_frame_.c_str(), base_frame_.c_str());
    const auto start_time = this->now();
    // 循环直到 ROS 关闭或变换可用
    while (rclcpp::ok())
    {
        try
        {
            // 尝试查找 map 到 base_link 的变换（TimePointZero 表示最新的变换）
            const auto transform_stamped = tf_buffer.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            (void)transform_stamped;
            // 如果没有抛出异常，说明变换可用
            RCLCPP_INFO(get_logger(), "TF %s -> %s is available.", map_frame_.c_str(), base_frame_.c_str());
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
            // 如果查找失败则短暂休眠后重试
            rclcpp::sleep_for(200ms);
        }

        if ((this->now() - start_time).seconds() > tf_wait_timeout_sec_)
        {
            RCLCPP_ERROR(get_logger(), "TF wait timeout after %.1f seconds.", tf_wait_timeout_sec_);
            return false;
        }
    }

    return false;
}

// 等待 action server 可用，返回布尔值
bool Nav2Client::wait_for_server()
{
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    // 等待 action server 最多 10 秒，返回是否可用
    const bool is_ready = m_ActionClient->wait_for_action_server(10s);
    return is_ready;
}

// 在仿真或使用 /use_sim_time 时，等待时间被发布（即时钟非 0）
bool Nav2Client::wait_for_time()
{
    // 说明：在仿真（use_sim_time=true）场景，若 /clock 尚未发布，ROS 时间会一直是 0。
    // 这里阻塞等待直到时间变为非 0；如果进程收到退出信号（rclcpp::ok()==false），则提前返回。
    const auto start_wall = std::chrono::steady_clock::now();
    while (true)
    {
        const bool is_ok = rclcpp::ok();
        if (false == is_ok)
        {
            return false;
        }

        const rcl_duration_value_t now_ns = this->get_clock()->now().nanoseconds();
        if (0 != now_ns)
        {
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() > clock_wait_timeout_sec_)
        {
            return false;
        }

        // 休眠避免空转占满 CPU
        rclcpp::sleep_for(100ms);
    }
}

// 发布初始位姿到 /initialpose，用于 AMCL 或 RViz 设置初始 pose
void Nav2Client::publish_initial_pose()
{
    // 构造消息类型 PoseWithCovarianceStamped
    PoseWithCovarianceStamped msg;
    // 设置坐标系为 map
    msg.header.frame_id = map_frame_;
    // 时间戳设为当前时间
    msg.header.stamp = this->now();
    // 设置初始位置 x, y
    msg.pose.pose.position.x = initial_x_;
    msg.pose.pose.position.y = initial_y_;

    // 构造四元数并设置为无旋转（roll/pitch/yaw 全为 0）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, initial_yaw_);
    // 把 tf2::Quaternion 转换为 ROS 消息的 quaternion
    msg.pose.pose.orientation = tf2::toMsg(q);

    // 设置协方差矩阵对角线上的三个值（x, y, yaw 的不确定度示例）
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.25;

    // 打印日志表示要发布初始位姿
    RCLCPP_INFO(get_logger(), "Publishing initial pose to /initialpose");
    // 连续发布几次，以便订阅方（如 AMCL）能接收到并应用初始位姿
    for (int i = 0; rclcpp::ok() && i < 5; ++i)
    {
        // 更新时间戳并发布消息
        msg.header.stamp = this->now();
        m_InitialPosePublisher->publish(msg);
        rclcpp::sleep_for(200ms);
    }
}

// 发送导航目标并返回 GoalHandle（或 nullptr 表示失败）
Nav2Client::GoalHandle::SharedPtr Nav2Client::send_goal()
{
    // 构造目标位姿，使用 map 作为参考坐标系
    PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = this->now();
    // 设置目标位置 x, y
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;

    // 设置目标朝向（无旋转）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal_yaw_);
    goal.pose.orientation = tf2::toMsg(q);

    // 构造 NavigateToPose 的 Goal 消息并赋值
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal;

    // 配置发送选项，包括反馈回调
    auto send_goal_options = NavigateClient::SendGoalOptions();
    // 目标响应回调：服务器会在接受/拒绝目标后回调一次。
    send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal rejected by server (goal_response_callback)");
            return;
        }
        RCLCPP_INFO(get_logger(), "Goal accepted by server (goal_response_callback)");
    };
    // 反馈回调：打印剩余距离（feedback->distance_remaining）
    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    { RCLCPP_INFO(get_logger(), "Remaining distance: %.2f", feedback->distance_remaining); };

    // 异步发送目标，返回一个 future（用于拿到 goal handle）
    auto goal_handle_future = m_ActionClient->async_send_goal(goal_msg, send_goal_options);

    // 等待发送 goal 的 future 完成，超时时间为 5 秒；如果失败则返回 nullptr
    const auto send_goal_code = rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future, 5s);
    if (send_goal_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Send goal call failed");
        return nullptr;
    }

    // 获取 GoalHandle（包含目标是否被接受等信息）
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        // 如果服务器拒绝目标，打印错误并返回 nullptr
        RCLCPP_ERROR(get_logger(),
                     "Goal was rejected by server. Common causes: Nav2 not fully active yet, "
                     "missing/invalid localization (initial pose), frame_id mismatch (goal frame='%s'), "
                     "or sim time mismatch (use_sim_time).",
                     map_frame_.c_str());
        return nullptr;
    }

    // 目标被接受，返回 GoalHandle，后续可以用来获取结果或取消
    RCLCPP_INFO(get_logger(), "Goal accepted, waiting for result...");
    return goal_handle;
}
