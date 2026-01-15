// ==============================================================================
// 任务编排节点 (TaskRunner) 实现文件
//
// 逻辑流程概览：
// 1. [初始化]：构造函数配置参数、建立 ROS 通信接口。
// 2. [环境自检]：等待仿真时钟、TF 坐标变换、Action 和 Service 准备就绪。
// 3. [配置加载]：从外部 YAML 文件读取机器人的巡检/任务路径点。
// 4. [任务循环]：
//    A. 去采集点 (Navigate to Pickup)
//    B. 执行抓取操作 (Call Pick Service)
//    C. 去投放点 (Navigate to Dropoff)
//    D. 执行放置操作 (Call Place Service)
// 5. [结束汇报]：所有任务完成后回到 Idle 状态并发布状态。
// ==============================================================================

#include "ros2_learning_task_runner/task_runner.hpp"

// --- TF2 坐标变换相关库 ---
// 四元数工具，用于处理机器人的朝向（2D 的偏航角在 ROS 3D 空间中需要转成四元数）
#include <tf2/LinearMath/Quaternion.h>
// 提供 PoseStamped 和 TF 变换消息之间的互转功能
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// Transform Buffer：缓存一段时间内机器人的所有坐标系变化，用于查询位置
#include <tf2_ros/buffer.h>
// Transform Listener：后台监听 /tf 话题，不断更新 Buffer
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <exception>

// 使用 chrono 库的便捷字面量（如 1s 代表 1秒），增加代码可读性
using namespace std::chrono_literals;

/**
 * @brief 构造函数实现
 *
 * 每一个 ROS 2 节点的生命周期都始于构造函数：
 * 步骤 1: 初始化父类 Node，设置节点的唯一识别名称为 "task_runner"。
 */
TaskRunner::TaskRunner()
        : Node("task_runner")
{
    // --- 第一步：声明并读取 ROS 参数 (Parameters) ---
    // 参数允许我们在不重新编译代码的情况下，通过 Launch
    // 文件或命令行动态修改节点行为。

    // (1) 坐标系参数：定义机器人参考的“大地图”和自身的“底盘中心”
    m_MapFrame = this->declare_parameter<std::string>("map_frame", "map");
    m_BaseFrame =
        this->declare_parameter<std::string>("base_frame", "base_link");

    // (2) 接口名称参数：定义导航 Action 和 抓取/放置 Service 的话题名称
    m_Nav2ActionName = this->declare_parameter<std::string>("nav2_action_name",
                                                            "navigate_to_pose");
    m_TaskConfigPath = this->declare_parameter<std::string>("task_config", "");

    m_PickServiceName = this->declare_parameter<std::string>(
        "pick_service", "/manipulation/pick");
    m_PlaceServiceName = this->declare_parameter<std::string>(
        "place_service", "/manipulation/place");

    // (3) 控制与超时参数：确保在资源不可用时节点不会无限阻塞
    m_ClockWaitTimeoutSec =
        this->declare_parameter<double>("clock_wait_timeout_sec", 20.0);
    m_TfWaitTimeoutSec =
        this->declare_parameter<double>("tf_wait_timeout_sec", 10.0);
    m_NavigationTimeoutSec =
        this->declare_parameter<double>("navigation_timeout_sec", 120.0);

    // (4) 初始位姿：用于在启动时通知 AMCL 机器人当前在地图的哪个位置
    m_InitialX = this->declare_parameter<double>("initial_x", 0.0);
    m_InitialY = this->declare_parameter<double>("initial_y", 0.0);
    m_InitialYaw = this->declare_parameter<double>("initial_yaw", 0.0);
    m_PublishInitialPose =
        this->declare_parameter<bool>("publish_initial_pose", false);

    // (5) 时钟源处理：如果是 Gazebo 仿真环境，需要同步仿真时间 (use_sim_time)
    if (this->has_parameter("use_sim_time"))
    {
        this->get_parameter("use_sim_time", m_UseSimTime);
    }
    else
    {
        m_UseSimTime = this->declare_parameter<bool>("use_sim_time", true);
    }

    // --- 第二步：创建 ROS 通信接口句柄 ---

    // 1. Action Client (动作客户端)
    // 动作 (Action)
    // 是一种长时任务。例如“导航”可能需要几分钟，期间需要有反馈，不能阻塞主线程。
    m_ActionClient =
        rclcpp_action::create_client<NavigateToPose>(this, m_Nav2ActionName);

    // 2. Service Client (服务客户端)
    // 服务 (Service)
    // 是一种快速、可靠的请求-响应。适合“抓取”、“放置”等结果明确的操作。
    m_PickClient = this->create_client<Trigger>(m_PickServiceName);
    m_PlaceClient = this->create_client<Trigger>(m_PlaceServiceName);

    // 3. Publisher (发布器)
    // 发布器允许我们向话题发送数据，所有人都可以订阅并查看。
    // 这里的 10 是“队列深度”，防止发送太快导致丢失。
    m_DistancePub = this->create_publisher<std_msgs::msg::Float32>(
        "distance_remaining", 10);
    m_StatePub =
        this->create_publisher<ros2_learning_task_runner::msg::TaskStatus>(
            "task_status", 10);
    m_InitialPosePub =
        this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

    // 初始化节点状态为 Idle (空闲)
    set_state(TaskState::kIdle, "init");
}

/**
 * @brief 执行任务逻辑 (Main Run Loop)
 *
 * 这是一个阻塞式的线性逻辑。在真实的 ROS 开发中，如果涉及多任务并发，可能会使用
 * Behavior Tree (行为树) 或 State Machine (状态机)。
 */
void TaskRunner::run()
{
    // --- 准备阶段 1: 仿真时钟 ---
    // 在 Gazebo 仿真中，如果没收到 /clock，rclcpp::now() 会一直停在 0 秒。
    if (m_UseSimTime)
    {
        RCLCPP_INFO(get_logger(), "正在等待仿真时钟 /clock...");
        if (!wait_for_time())
        {
            set_state(TaskState::kFailed, "wait_for_time", "等待时钟超时");
            return;
        }
    }

    // --- 准备阶段 2: 加载配置文件 ---
    // 读取保存了所有位姿信息的 YAML 文件。
    if (!load_task_config(m_TaskConfigPath))
    {
        set_state(TaskState::kFailed, "load_task_config", "配置文件解析失败");
        return;
    }
    m_PickupTotal = static_cast<uint32_t>(m_PickupPoints.size());

    // --- 准备阶段 3: 等待核心服务上线 ---
    // 必须要等到 Nav2 导航服务器和机械臂服务都准备好，任务才能开始。
    if (!wait_for_action_server())
    {
        set_state(TaskState::kFailed, "wait_for_action_server",
                  "导航服务未启动");
        return;
    }

    // 如果开启了初始位姿发布（帮助 AMCL 定位），则执行一次发布操作。
    if (m_PublishInitialPose)
    {
        publish_initial_pose();
    }

    // 坐标系检查：检查地图 (map) 到 机器人 (base_link) 的数学变换链是否打通。
    if (!wait_for_tf())
    {
        set_state(TaskState::kFailed, "wait_for_tf", "坐标变换(TF)不可用");
        return;
    }

    // 检查抓取和放置服务是否在线。
    if (!wait_for_service(m_PickClient, m_PickServiceName) ||
        !wait_for_service(m_PlaceClient, m_PlaceServiceName))
    {
        set_state(TaskState::kFailed, "wait_for_services", "机械臂服务未发现");
        return;
    }

    // 任务数据完整性检查
    if (!m_HasDropoff || m_PickupPoints.empty())
    {
        set_state(TaskState::kFailed, "validate_task_config",
                  "配置中缺失点位信息");
        return;
    }

    // --- 核心执行阶段：任务循环 ---
    // 循环遍历每一个采集点，依次执行任务。
    for (size_t i = 0; rclcpp::ok() && i < m_PickupPoints.size(); ++i)
    {
        const auto &pickup = m_PickupPoints[i];
        m_CurrentPickupIndex = static_cast<uint32_t>(i + 1);

        // [阶段 A]：导航前往当前的采集点 (Pickup)
        set_state(TaskState::kGoingToPickup, "前往采集点");
        if (!navigate_to(pickup, "pickup"))
        {
            set_state(TaskState::kFailed, "navigate_pickup",
                      "去采集点路途中失败");
            continue; // 如果这一点导航失败，由于机器人安全考虑，通常跳过而非报错停止
        }

        // [阶段 B]：抵达采集点，调用机械臂抓取服务 (Pick)
        set_state(TaskState::kPicking, "执行抓取");
        if (!call_trigger(m_PickClient, "pick"))
        {
            set_state(TaskState::kFailed, "pick", "抓取动作执行失败");
            continue;
        }

        // [阶段 C]：带货前往投放点 (Dropoff)
        set_state(TaskState::kGoingToDropoff, "前往投放点");
        if (!navigate_to(m_Dropoff, "dropoff"))
        {
            set_state(TaskState::kFailed, "navigate_dropoff",
                      "去投放点路途中失败");
            continue;
        }

        // [阶段 D]：抵达投放点，调用机械臂放置服务 (Place)
        set_state(TaskState::kPlacing, "执行放置");
        if (!call_trigger(m_PlaceClient, "place"))
        {
            set_state(TaskState::kFailed, "place", "放置动作执行失败");
            continue;
        }
    }

    // 所有点位任务完成后，重置索引并设置为 Idle
    m_CurrentPickupIndex = 0;
    set_state(TaskState::kIdle, "任务圆满完成");
    RCLCPP_INFO(get_logger(), "Task runner completed.");
}

/**
 * @brief 状态机管理与发布函数
 *
 * 每当任务进度发生变化（如切换阶段、遇到错误），都必须通过此函数同步内部变量并广播状态。
 */
void TaskRunner::set_state(TaskState state, const std::string &phase,
                           const std::string &error)
{
    m_CurrentState = state;
    m_CurrentPhase = phase;

    // 如果存在具体的错误信息，则记录下来
    if (!error.empty())
    {
        m_LastError = error;
    }
    else if (state != TaskState::kFailed)
    {
        // 只有在非失败状态下才清除旧的错误记录
        m_LastError.clear();
    }

    // 填充 ROS 消息并发布给监听者（如 Web UI 或 Dashboard 节点）
    ros2_learning_task_runner::msg::TaskStatus msg;
    msg.state = state_to_string(state);
    msg.phase = m_CurrentPhase;
    msg.pickup_index = m_CurrentPickupIndex;
    msg.pickup_total = m_PickupTotal;
    msg.last_error = m_LastError;
    m_StatePub->publish(msg);
}

/**
 * @brief 内部辅助：将枚举状态转换为易读的字符串流
 */
std::string TaskRunner::state_to_string(TaskState state) const
{
    switch (state)
    {
    case TaskState::kIdle:
        return "空闲(Idle)";
    case TaskState::kGoingToPickup:
        return "前往采集点(GoingToPickup)";
    case TaskState::kPicking:
        return "执行抓取(Picking)";
    case TaskState::kGoingToDropoff:
        return "前往投放点(GoingToDropoff)";
    case TaskState::kPlacing:
        return "执行放置(Placing)";
    case TaskState::kFailed:
        return "任务失败(Failed)";
    default:
        return "未知(Unknown)";
    }
}

/**
 * @brief YAML 配置文件加载逻辑 (使用 yaml-cpp)
 */
bool TaskRunner::load_task_config(const std::string &path)
{
    if (path.empty())
    {
        RCLCPP_ERROR(get_logger(), "配置路径参数为空，无法加载任务清单。");
        return false;
    }

    YAML::Node root;
    try
    {
        // 1. 打开并解析物理文件
        root = YAML::LoadFile(path);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "YAML 语法解析错误: %s", e.what());
        return false;
    }

    // 2. 检查 YAML 中是否有覆盖全局坐标系的设定
    if (root["map_frame"])
    {
        m_MapFrame = root["map_frame"].as<std::string>();
    }

    // 3. 解析核心任务点：投放点 (Dropoff)
    if (!parse_pose_node(root["dropoff"], "投放点(dropoff)", &m_Dropoff))
        return false;
    m_HasDropoff = true;

    // 4. 解析核心任务点列表：采集点序列 (Pickups)
    m_PickupPoints.clear();
    const auto pickups = root["pickups"];
    if (!pickups || !pickups.IsSequence()) // 确保它是一个列表格式
    {
        RCLCPP_ERROR(get_logger(), "YAML 配置中缺失 'pickups' 列表。");
        return false;
    }

    for (size_t i = 0; i < pickups.size(); ++i)
    {
        Pose2D pose;
        if (!parse_pose_node(pickups[i], "采集点(pickup)", &pose))
            return false;
        m_PickupPoints.push_back(pose);
    }

    return true;
}

/**
 * @brief 从单个 YAML 节点中提取 x, y 和偏航角 yaw
 */
bool TaskRunner::parse_pose_node(const YAML::Node &node,
                                 const std::string &label, Pose2D *out_pose)
{
    if (!node || !out_pose)
        return false;

    // x 和 y 是机器人在地图上的基础坐标，不能为空
    if (!node["x"] || !node["y"])
    {
        RCLCPP_ERROR(get_logger(), "%s 配置缺失必要的 'x' 或 'y'。",
                     label.c_str());
        return false;
    }

    out_pose->x = node["x"].as<double>();
    out_pose->y = node["y"].as<double>();
    // yaw 是可选的（面向方向），如果没提供默认为 0.0 (朝向 X 轴正向)
    out_pose->yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
    return true;
}

/**
 * @brief 系统时钟等待：解决“now() 为 0”导致的定时器或 TF 失败问题
 */
bool TaskRunner::wait_for_time()
{
    // 这里使用 Wall Time (真实世界的挂钟时间) 进行超时判断
    const auto start_wall = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        // 当 ROS 2 的接口能够返回大于 0 的纳秒数时，说明时钟已通。
        if (this->get_clock()->now().nanoseconds() != 0)
            return true;

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > m_ClockWaitTimeoutSec)
        {
            return false;
        }
        rclcpp::sleep_for(100ms); // 短暂休息，防止 CPU 负载过高
    }
    return false;
}

/**
 * @brief TF 坐标系链路检查：确保定位系统 (如 AMCL 或 SLAM) 已经发布了位姿
 */
bool TaskRunner::wait_for_tf()
{
    // 实例化一个本地的 TF2 缓冲和监听器来专门探测链路
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    RCLCPP_INFO(get_logger(), "正在等待 TF 坐标链路就绪: [%s] -> [%s]...",
                m_MapFrame.c_str(), m_BaseFrame.c_str());

    const auto start_time = this->now();
    while (rclcpp::ok())
    {
        try
        {
            // lookupTransform 会尝试查找两个坐标系之间的变换矩阵
            // 如果链条未打通，它会抛出异常
            tf_buffer.lookupTransform(m_MapFrame, m_BaseFrame,
                                      tf2::TimePointZero);
            RCLCPP_INFO(get_logger(), "TF 链路探测成功，可以开始导航。");
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            // 异常捕获机制：如果没通，等待 200ms 后重试
            rclcpp::sleep_for(200ms);
        }

        // 检查逻辑时间是否超时
        if ((this->now() - start_time).seconds() > m_TfWaitTimeoutSec)
            return false;
    }
    return false;
}

/**
 * @brief 导航服务端等待：确保 Nav2 模块已准备好接受控制指令
 */
bool TaskRunner::wait_for_action_server()
{
    RCLCPP_INFO(get_logger(), "正在连接 Nav2 Action 服务端 [%s]...",
                m_Nav2ActionName.c_str());
    return m_ActionClient->wait_for_action_server(10s);
}

/**
 * @brief 通用 Service 服务端等待
 */
bool TaskRunner::wait_for_service(
    const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name)
{
    RCLCPP_INFO(get_logger(), "正在连接服务 [%s]...", name.c_str());
    return client->wait_for_service(10s);
}

/**
 * @brief 数据格式转换：从简洁的 Pose2D 转为标准的 ROS PoseStamped 消息
 */
TaskRunner::PoseStamped TaskRunner::make_pose(const Pose2D &pose) const
{
    PoseStamped msg;
    // Header 包含了坐标系 (Frame) 和 采样时间 (Stamp)
    msg.header.frame_id = m_MapFrame;
    msg.header.stamp = this->now();

    // Position 包含 XYZ 三轴坐标
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;

    // Orientation (姿态) 在 ROS 消息中是以四元数 (x, y, z, w) 形式存储的。
    // 我们必须将角度 (yaw) 转换成这种格式，导航算法才能识别。
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw); // 设定 Roll, Pitch 为 0，仅设定 Yaw
    msg.pose.orientation = tf2::toMsg(q);

    return msg;
}

/**
 * @brief 初始位姿推送：常用于系统刚启动时通知 AMCL 机器人的大致位置
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

    // 设置协方差矩阵：这里定义一个大致的置信区域，矩阵对角线设定为 0.25
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.25;

    RCLCPP_INFO(get_logger(), "正在向 /initialpose 发布机器人初识位姿...");
    for (int i = 0; rclcpp::ok() && i < 5; ++i)
    {
        msg.header.stamp = this->now();
        m_InitialPosePub->publish(msg);
        rclcpp::sleep_for(200ms); // 连续发布多次以确保 RVIZ 或 AMCL 节点收到
    }
}

/**
 * @brief 导航到目标点 (核心 Action 逻辑)
 *
 * 对于新手最难点的解析：
 * 我们采用的是异步调用模式，即：发通知 -> 等确认 -> 等结果。
 */
bool TaskRunner::navigate_to(const Pose2D &pose, const std::string &label)
{
    // 1. 组装并发送 Goal (目标请求)
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_pose(pose);

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // [回调机制]：当导航组件有反馈数据时（比如还剩多少米），会触发此匿名函数。
    send_goal_options.feedback_callback =
        [this,
         label](GoalHandle::SharedPtr,
                const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // 反馈：将剩余距离发布到话题中，供外部（如手机 APP）展示。
        std_msgs::msg::Float32 msg;
        msg.data = feedback->distance_remaining;
        m_DistancePub->publish(msg);

        // 降低日志频率，每 2 秒打印一次，防止控制台刷屏。
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[%s] 剩余距离: %.2f 米", label.c_str(),
                             feedback->distance_remaining);
    };

    // 2. 将目标异步发送给 Nav2 模块。
    auto goal_future =
        m_ActionClient->async_send_goal(goal_msg, send_goal_options);

    // 手动推进 ROS 事件循环，等待服务器“确认接收”目标（非阻塞等待）。
    const auto send_code =
        rclcpp::spin_until_future_complete(shared_from_this(), goal_future, 5s);
    if (send_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "[%s] 目标发送失败，服务器可能未响应。",
                     label.c_str());
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(),
                     "[%s] 导航目标被服务器拒绝（路径可能不通）。",
                     label.c_str());
        return false;
    }

    // 3. 等待机器人最终执行结果（成功抵达 vs 时间超时）。
    auto result_future = m_ActionClient->async_get_result(goal_handle);
    const std::chrono::duration<double> timeout(m_NavigationTimeoutSec);
    const auto result_code = rclcpp::spin_until_future_complete(
        shared_from_this(), result_future, timeout);

    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto wrapped_result = result_future.get();
        // 返回执行是否成功 (SUCCEEDED 为唯一成功态)
        return (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }

    // [容错处理]：如果导航超时（比如机器人被搬走、路被堵死），必须主动“取消”目标。
    RCLCPP_WARN(get_logger(), "[%s] 导航超时，正在向组件发送取消指令...",
                label.c_str());
    auto cancel_future = m_ActionClient->async_cancel_goal(goal_handle);
    (void)rclcpp::spin_until_future_complete(shared_from_this(), cancel_future,
                                             5s);
    return false;
}

/**
 * @brief 通用 Service 调用模板（适用于 Trigger 类型服务）
 */
bool TaskRunner::call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client,
                              const std::string &name)
{
    // 构造请求数据 (Trigger 类型消息没有任何输入参数)
    auto request = std::make_shared<Trigger::Request>();

    RCLCPP_INFO(get_logger(), "正在调用 [%s] 执行具体动作...", name.c_str());

    // 异步发送请求并返回一个“未来凭据 (future)”
    auto future = client->async_send_request(request);

    // 阻塞当前线程，等待响应（最多等待 10s）
    const auto result_code =
        rclcpp::spin_until_future_complete(shared_from_this(), future, 10s);
    if (result_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "服务调用超时或失败: %s", name.c_str());
        return false;
    }

    // 从 future 中取出真正的响应对象内容
    const auto response = future.get();
    if (!response->success)
    {
        RCLCPP_WARN(get_logger(), "服务反馈失败: %s, 错误详情: %s",
                    name.c_str(), response->message.c_str());
    }
    return response->success;
}
