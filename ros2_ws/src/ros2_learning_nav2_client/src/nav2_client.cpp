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
        , m_InitialPosePublisher() // 延后在构造函数体内 create_publisher
        , m_ActionClient()         // 延后在构造函数体内 create_client
        , m_GetStateClient()
        , m_ManageNodesClient()
        , m_AmclPoseSub() // 延后在构造函数体内 create_subscription
        , have_amcl_pose_(false)
        , amcl_pose_mutex_()
        , last_amcl_pose_()
        // 下面这批是“参数默认值”，会在构造函数体内通过 declare_parameter
        // 允许被覆盖。
        , map_frame_("map")
        , base_frame_("base_link")
        , initial_x_(0.0)
        , initial_y_(0.0)
        , initial_yaw_(0.0)
        , goal_x_(1.0)
        , goal_y_(0.0)
        , goal_yaw_(0.0)
        , tf_wait_timeout_sec_(10.0)
        , clock_wait_timeout_sec_(20.0)
        , nav2_active_timeout_sec_(20.0)
        , amcl_pose_timeout_sec_(10.0)
        , use_sim_time_(false)
        , wait_nav2_active_(true)
        , wait_amcl_pose_(true)
        , auto_startup_nav2_(true)
        , lifecycle_get_state_service_("/bt_navigator/get_state")
        , lifecycle_manage_nodes_service_(
              "/lifecycle_manager_navigation/manage_nodes")
        , amcl_pose_topic_("/amcl_pose")
{
    // 参数声明与读取：允许通过 ROS 参数覆盖默认值
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ =
        this->declare_parameter<std::string>("base_frame", "base_link");
    initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);
    goal_x_ = this->declare_parameter<double>("goal_x", 1.0);
    goal_y_ = this->declare_parameter<double>("goal_y", 0.0);
    goal_yaw_ = this->declare_parameter<double>("goal_yaw", 0.0);

    tf_wait_timeout_sec_ =
        this->declare_parameter<double>("tf_wait_timeout_sec", 10.0);
    clock_wait_timeout_sec_ =
        this->declare_parameter<double>("clock_wait_timeout_sec", 20.0);

    lifecycle_get_state_service_ = this->declare_parameter<std::string>(
        "lifecycle_get_state_service", "/bt_navigator/get_state");
    lifecycle_manage_nodes_service_ = this->declare_parameter<std::string>(
        "lifecycle_manage_nodes_service",
        "/lifecycle_manager_navigation/manage_nodes");
    nav2_active_timeout_sec_ =
        this->declare_parameter<double>("nav2_active_timeout_sec", 20.0);
    wait_nav2_active_ = this->declare_parameter<bool>("wait_nav2_active", true);
    auto_startup_nav2_ =
        this->declare_parameter<bool>("auto_startup_nav2", true);

    amcl_pose_topic_ =
        this->declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    amcl_pose_timeout_sec_ =
        this->declare_parameter<double>("amcl_pose_timeout_sec", 10.0);
    wait_amcl_pose_ = this->declare_parameter<bool>("wait_amcl_pose", true);

    // 默认 true：本仓库的 nav2_client 主要用于 Gazebo/Nav2
    // 仿真栈，若不启用仿真时间， 发送 initialpose/goal
    // 时的时间戳会是系统时间，可能导致 Nav2 侧判定异常并 ABORT。 注意：rclcpp
    // 可能会在内部自动声明 use_sim_time（TimeSource）。 因此这里要避免重复
    // declare 导致: "parameter 'use_sim_time' has already been declared"。
    if (!this->has_parameter("use_sim_time"))
    {
        use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
    }
    else
    {
        this->get_parameter("use_sim_time", use_sim_time_);
    }

    // lifecycle 状态查询客户端：用于观察 Nav2 栈是否进入 ACTIVE
    m_GetStateClient = this->create_client<lifecycle_msgs::srv::GetState>(
        lifecycle_get_state_service_);
    // lifecycle manager 客户端：用于一键启动/激活整个导航栈（可选）
    m_ManageNodesClient =
        this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            lifecycle_manage_nodes_service_);

    // 订阅 AMCL
    // 位姿：当定位模块稳定输出位姿时，代表机器人已在地图坐标系中“定位成功”
    m_AmclPoseSub = this->create_subscription<PoseWithCovarianceStamped>(
        amcl_pose_topic_, 10,
        [this](const PoseWithCovarianceStamped::SharedPtr msg)
        {
            // 保存最新位姿，后续用于打印或诊断定位是否正常
            {
                std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
                last_amcl_pose_ = *msg;
            }
            // 标记定位已就绪，允许继续发导航目标
            have_amcl_pose_.store(true, std::memory_order_release);
        });

    // 创建一个发布器，用于发布初始位姿到 /initialpose 话题，队列深度为 10
    m_InitialPosePublisher =
        this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);
    // 创建一个 action 客户端，用来与 Nav2 的 navigate_to_pose action 通信
    m_ActionClient =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

// 入口运行函数：负责等待时间、等待 action
// server、发布初始位姿、发送目标并等待结果
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
                         "Start the simulation/Nav2 stack first, or run with "
                         "-p use_sim_time:=false.",
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
            RCLCPP_WARN(
                get_logger(),
                "Nav2 stack not active within timeout; proceeding anyway.");
        }
    }

    // 发布初始位姿到 /initialpose，便于在 RViz 或 AMCL 中设置初始位姿
    publish_initial_pose();

    // 等待 AMCL 位姿（确保 initialpose 被处理；否则 BT Navigator 可能拒绝
    // goal）
    if (wait_amcl_pose_)
    {
        const bool amcl_ok = wait_for_amcl_pose();
        if (!amcl_ok)
        {
            RCLCPP_WARN(get_logger(),
                        "No AMCL pose received within timeout; continuing.");
        }
    }

    // 等待 TF 中 map->base_link 的变换可用，确保里程计/定位信息就绪
    const bool is_tf_ready = wait_for_tf();
    if (!is_tf_ready)
    {
        RCLCPP_ERROR(get_logger(),
                     "TF is not available within timeout, aborting.");
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
    const auto result_code = rclcpp::spin_until_future_complete(
        shared_from_this(), result_future, 60s);

    // 如果等待成功（future 返回），处理结果码
    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        // 获取封装的结果对象
        const auto wrapped_result = result_future.get();
        const auto action_result_code = wrapped_result.code;
        const auto result = wrapped_result.result;
        const auto error_code = result ? result->error_code : 0;
        const auto error_msg = (result && !result->error_msg.empty())
                                   ? result->error_msg.c_str()
                                   : "";
        // 根据 action 返回的 code 进行不同日志输出
        switch (action_result_code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // 导航成功
            RCLCPP_INFO(get_logger(),
                        "Navigation succeeded. error_code=%u error_msg='%s'",
                        error_code, error_msg);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            // 导航被中止
            RCLCPP_WARN(get_logger(),
                        "Navigation aborted. error_code=%u error_msg='%s'",
                        error_code, error_msg);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            // 导航被取消
            RCLCPP_WARN(get_logger(),
                        "Navigation canceled. error_code=%u error_msg='%s'",
                        error_code, error_msg);
            break;
        default:
            // 未知结果码
            RCLCPP_WARN(
                get_logger(),
                "Unknown result code. code=%d error_code=%u error_msg='%s'",
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
            RCLCPP_WARN(get_logger(),
                        "ROS is shutting down; skip cancel request.");
            return;
        }

        auto cancel_future = m_ActionClient->async_cancel_goal(goal_handle);
        // 等待取消操作完成，最长等待 5 秒
        const auto cancel_code = rclcpp::spin_until_future_complete(
            shared_from_this(), cancel_future, 5s);
        (void)cancel_code;
    }
    else
    {
        // 例如 Ctrl-C 中断：rclcpp::FutureReturnCode::INTERRUPTED
        RCLCPP_WARN(get_logger(), "Navigation wait interrupted.");
        return;
    }
}

void Nav2Client::spin_some_for(std::chrono::nanoseconds duration)
{
    // 使用单线程执行器处理回调，确保在等待期间订阅/服务响应仍能被处理
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(shared_from_this());

    // 以真实时间计时，避免受 ROS 时间（仿真时钟）影响
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        // 主动执行一次回调（订阅 /amcl_pose、服务返回等）
        exec.spin_some();

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= duration)
        {
            break;
        }

        // 适当 sleep，避免 while 空转占用过多 CPU
        std::this_thread::sleep_for(20ms);
    }
}

bool Nav2Client::is_lifecycle_active(uint8_t state_id) const
{
    // ACTIVE 表示 Nav2 行为树可响应目标，机器人具备执行导航任务的能力
    return state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool Nav2Client::is_lifecycle_inactive(uint8_t state_id) const
{
    // INACTIVE 表示 Nav2 已配置但未激活，常见于刚启动或启动未完成的状态
    return state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

void Nav2Client::log_nav2_state_throttled(
    uint8_t state_id, const std::string &state_label,
    std::chrono::steady_clock::time_point &last_log) const
{
    // 使用 steady_clock 做节流，保证日志频率稳定，不受仿真时间影响
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log < 1s)
    {
        return;
    }

    RCLCPP_INFO(get_logger(), "Nav2 state: id=%u label='%s' (%s)", state_id,
                state_label.c_str(), lifecycle_get_state_service_.c_str());
    last_log = now;
}

std::string Nav2Client::get_change_state_service_from_get_state_service() const
{
    // 默认值指向 bt_navigator（Nav2 的行为树节点），符合 Nav2 的典型默认命名
    std::string change_state_service = "/bt_navigator/change_state";

    const std::string suffix = "/get_state";
    if (lifecycle_get_state_service_.size() > suffix.size() &&
        lifecycle_get_state_service_.compare(
            lifecycle_get_state_service_.size() - suffix.size(), suffix.size(),
            suffix) == 0)
    {
        // 截取节点名（去掉 /get_state 后缀）并拼出 /change_state 服务名
        const std::string node_name = lifecycle_get_state_service_.substr(
            0, lifecycle_get_state_service_.size() - suffix.size());
        change_state_service = node_name + "/change_state";
    }

    return change_state_service;
}

bool Nav2Client::try_lifecycle_manager_startup()
{
    // 生命周期管理器用于统一启动/激活 Nav2 多个节点，适合完整导航栈的标准流程
    if (!m_ManageNodesClient)
    {
        return false;
    }

    // 避免在服务尚未起来时阻塞过久
    if (!m_ManageNodesClient->wait_for_service(1s))
    {
        RCLCPP_WARN(get_logger(), "Lifecycle manager service not available: %s",
                    lifecycle_manage_nodes_service_.c_str());
        return false;
    }

    auto req =
        std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    req->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
    auto fut = m_ManageNodesClient->async_send_request(req);
    const auto rc =
        rclcpp::spin_until_future_complete(shared_from_this(), fut, 5s);
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(get_logger(),
                    "Failed to call lifecycle manager STARTUP (%s)",
                    lifecycle_manage_nodes_service_.c_str());
        return false;
    }

    const auto resp = fut.get();
    const bool ok = resp && resp->success;
    // 记录启动结果，便于现场诊断“导航无法接收目标”的根因
    RCLCPP_WARN(get_logger(),
                "Sent STARTUP to lifecycle manager (%s): success=%s",
                lifecycle_manage_nodes_service_.c_str(), ok ? "true" : "false");
    return ok;
}

bool Nav2Client::try_activate_lifecycle_node()
{
    // 直接激活目标 lifecycle 节点（例如 bt_navigator），作为 lifecycle manager
    // 不可用时的兜底
    const std::string change_state_service =
        get_change_state_service_from_get_state_service();

    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
        change_state_service);
    if (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(get_logger(),
                    "Lifecycle change_state service not available: %s",
                    change_state_service.c_str());
        return false;
    }

    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    auto fut = client->async_send_request(req);
    const auto rc =
        rclcpp::spin_until_future_complete(shared_from_this(), fut, 5s);
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(get_logger(), "Direct ACTIVATE call did not complete (%s)",
                    change_state_service.c_str());
        return false;
    }

    const auto resp = fut.get();
    const bool ok = resp && resp->success;
    // 这里用 WARN 级别打印，提示用户导航栈可能还未完全就绪
    RCLCPP_WARN(get_logger(), "Tried direct ACTIVATE (%s): success=%s",
                change_state_service.c_str(), ok ? "true" : "false");
    return ok;
}

void Nav2Client::ensure_nav2_active_best_effort()
{
    // 先尝试 manager 的统一启动；失败时再直接激活，尽力让导航栈可用
    const bool startup_ok = try_lifecycle_manager_startup();
    if (!startup_ok)
    {
        (void)try_activate_lifecycle_node();
    }
}

bool Nav2Client::wait_for_nav2_active()
{
    // 确保 get_state 客户端已创建，否则无法查询 Nav2 生命周期状态
    if (!m_GetStateClient)
    {
        return false;
    }

    // 等待 get_state 服务出现，避免过早发送请求导致报错
    RCLCPP_INFO(get_logger(),
                "Waiting for Nav2 lifecycle get_state service: %s",
                lifecycle_get_state_service_.c_str());
    if (!m_GetStateClient->wait_for_service(5s))
    {
        RCLCPP_WARN(get_logger(), "Service not available: %s",
                    lifecycle_get_state_service_.c_str());
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    auto last_log = start;
    // startup_sent 防止重复启动/激活，避免触发 Nav2 过多状态切换
    bool startup_sent = false;
    while (rclcpp::ok())
    {
        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = m_GetStateClient->async_send_request(req);

        const auto rc =
            rclcpp::spin_until_future_complete(shared_from_this(), future, 1s);
        if (rc == rclcpp::FutureReturnCode::SUCCESS)
        {
            const auto resp = future.get();
            if (resp)
            {
                // 读取当前状态并定期记录，便于用户判断卡在哪个阶段
                log_nav2_state_throttled(resp->current_state.id,
                                         resp->current_state.label, last_log);

                if (is_lifecycle_active(resp->current_state.id))
                {
                    RCLCPP_INFO(get_logger(),
                                "Nav2 lifecycle node is ACTIVE (%s)",
                                lifecycle_get_state_service_.c_str());
                    return true;
                }

                // 若处于 INACTIVE，可选择只尝试一次启动/激活
                if (!startup_sent && auto_startup_nav2_ &&
                    is_lifecycle_inactive(resp->current_state.id))
                {
                    // 对真实机器人：避免长时间处于 INACTIVE 导致导航目标被拒
                    ensure_nav2_active_best_effort();
                    startup_sent = true;
                }
            }
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        // 超时退出，避免无限等待；业务上允许继续尝试发目标（但可能被拒）
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > nav2_active_timeout_sec_)
        {
            return false;
        }

        // 频率控制，避免对服务端造成过高请求压力
        std::this_thread::sleep_for(200ms);
    }

    return false;
}

bool Nav2Client::wait_for_amcl_pose()
{
    // 未配置 AMCL 位姿话题时直接返回
    if (amcl_pose_topic_.empty())
    {
        return false;
    }

    // 若已经收到过 AMCL 位姿，可直接认为定位就绪
    if (have_amcl_pose_.load(std::memory_order_acquire))
    {
        RCLCPP_INFO(get_logger(), "AMCL pose already received on %s",
                    amcl_pose_topic_.c_str());
        return true;
    }

    RCLCPP_INFO(get_logger(), "Waiting for AMCL pose on %s...",
                amcl_pose_topic_.c_str());

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        // 处理订阅回调，等待 /amcl_pose 到来
        spin_some_for(200ms);
        if (have_amcl_pose_.load(std::memory_order_acquire))
        {
            std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
            // 打印最近一次定位结果，便于核对机器人初始落点
            RCLCPP_INFO(get_logger(), "AMCL pose received: (%.3f, %.3f)",
                        last_amcl_pose_.pose.pose.position.x,
                        last_amcl_pose_.pose.pose.position.y);
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        // 超时放弃等待，允许继续流程（可能导致导航拒绝或路径异常）
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > amcl_pose_timeout_sec_)
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
    RCLCPP_INFO(get_logger(), "Waiting for TF %s -> %s...", map_frame_.c_str(),
                base_frame_.c_str());
    const auto start_time = this->now();
    // 循环直到 ROS 关闭或变换可用
    while (rclcpp::ok())
    {
        try
        {
            // 尝试查找 map 到 base_link 的变换（TimePointZero 表示最新的变换）
            const auto transform_stamped = tf_buffer.lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
            (void)transform_stamped;
            // 如果没有抛出异常，说明变换可用
            RCLCPP_INFO(get_logger(), "TF %s -> %s is available.",
                        map_frame_.c_str(), base_frame_.c_str());
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
            RCLCPP_ERROR(get_logger(), "TF wait timeout after %.1f seconds.",
                         tf_wait_timeout_sec_);
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
    // 说明：在仿真（use_sim_time=true）场景，若 /clock 尚未发布，ROS
    // 时间会一直是 0。 这里阻塞等待直到时间变为非
    // 0；如果进程收到退出信号（rclcpp::ok()==false），则提前返回。
    const auto start_wall = std::chrono::steady_clock::now();
    while (true)
    {
        const bool is_ok = rclcpp::ok();
        if (false == is_ok)
        {
            return false;
        }

        const rcl_duration_value_t now_ns =
            this->get_clock()->now().nanoseconds();
        if (0 != now_ns)
        {
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > clock_wait_timeout_sec_)
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
    send_goal_options.goal_response_callback =
        [this](GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(),
                         "Goal rejected by server (goal_response_callback)");
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Goal accepted by server (goal_response_callback)");
    };
    // 反馈回调：打印剩余距离（feedback->distance_remaining）
    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Remaining distance: %.2f",
                    feedback->distance_remaining);
    };

    // 异步发送目标，返回一个 future（用于拿到 goal handle）
    auto goal_handle_future =
        m_ActionClient->async_send_goal(goal_msg, send_goal_options);

    // 等待发送 goal 的 future 完成，超时时间为 5 秒；如果失败则返回 nullptr
    const auto send_goal_code = rclcpp::spin_until_future_complete(
        shared_from_this(), goal_handle_future, 5s);
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
                     "Goal was rejected by server. Common causes: Nav2 not "
                     "fully active yet, "
                     "missing/invalid localization (initial pose), frame_id "
                     "mismatch (goal frame='%s'), "
                     "or sim time mismatch (use_sim_time).",
                     map_frame_.c_str());
        return nullptr;
    }

    // 目标被接受，返回 GoalHandle，后续可以用来获取结果或取消
    RCLCPP_INFO(get_logger(), "Goal accepted, waiting for result...");
    return goal_handle;
}
