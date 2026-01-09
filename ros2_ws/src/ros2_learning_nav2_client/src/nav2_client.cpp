// 包含本包对应的头文件，声明了 Nav2Client 类与相关类型
#include "ros2_learning_nav2_client/nav2_client.hpp"

// 引入 tf2 相关异常类型（用于捕获 transform 查找中的异常）
#include <tf2/exceptions.h>
// 引入 TF2 的 Buffer，用来缓存和查询坐标变换
#include <tf2_ros/buffer.h>
// 引入 TF2 的 TransformListener，用于订阅并把变换写入 Buffer
#include <tf2_ros/transform_listener.h>

// 使用 std::chrono 的字面量（如 100ms, 1s 等）方便写时长常量
using namespace std::chrono_literals;

// 构造函数：初始化为一个 rclcpp 节点，节点名为 "nav2_minimal_client"
Nav2Client::Nav2Client()
        : Node("nav2_minimal_client")
{
    // 以下两行被注释掉了：用于在参数服务器声明并设置 use_sim_time
    // this->declare_parameter<bool>("use_sim_time", true);
    // sthis->set_parameter(rclcpp::Parameter("use_sim_time", this->get_parameter("use_sim_time").as_bool()));

    // 创建一个发布器，用于发布初始位姿到 /initialpose 话题，队列深度为 10
    mInitialPosePublisher = this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);
    // 创建一个 action 客户端，用来与 Nav2 的 navigate_to_pose action 通信
    mActionClient = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

// 入口运行函数：负责等待时间、等待 action server、发布初始位姿、发送目标并等待结果
void Nav2Client::run()
{
    // 确保系统时间已经可用（尤其在仿真中常需要等待 /use_sim_time 更新）
    wait_for_time();

    // 等待 Nav2 的 action server 可用，如果不可用则报错并返回
    const bool is_server_ready = wait_for_server();
    if (!is_server_ready)
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable.");
        return;
    }

    // 发布初始位姿到 /initialpose，便于在 RViz 或 AMCL 中设置初始位姿
    publish_initial_pose();
    // 等待 TF 中 map->base_link 的变换可用，确保里程计/定位信息就绪
    wait_for_tf();

    // 发送导航目标并获取 goal_handle
    auto goal_handle = send_goal();
    // 如果发送失败（返回空句柄），打印错误并返回
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send goal.");
        return;
    }

    // 异步获取结果的 future
    auto result_future = mActionClient->async_get_result(goal_handle);
    // 等待结果，最长等待 60 秒
    const auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), result_future, 60s);

    // 如果等待成功（future 返回），处理结果码
    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        // 获取封装的结果对象
        const auto wrapped_result = result_future.get();
        const auto action_result_code = wrapped_result.code;
        // 根据 action 返回的 code 进行不同日志输出
        switch (action_result_code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // 导航成功
            RCLCPP_INFO(get_logger(), "Navigation succeeded.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            // 导航被中止
            RCLCPP_WARN(get_logger(), "Navigation aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            // 导航被取消
            RCLCPP_WARN(get_logger(), "Navigation canceled.");
            break;
        default:
            // 未知结果码
            RCLCPP_WARN(get_logger(), "Unknown result code.");
            break;
        }
    }
    else
    {
        // 如果等待超时或 future 未成功返回，则尝试取消目标
        RCLCPP_WARN(get_logger(), "Navigation timeout, canceling goal.");
        auto cancel_future = mActionClient->async_cancel_goal(goal_handle);
        // 等待取消操作完成，最长等待 5 秒
        const auto cancel_code = rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
        (void)cancel_code;
    }
}

// 等待 TF 中 map -> base_link 变换可用的函数
void Nav2Client::wait_for_tf()
{
    // 使用节点的时钟构造一个 tf2 buffer，用来缓存变换
    tf2_ros::Buffer tf_buffer(this->get_clock());
    // 构造 TransformListener，它会订阅 /tf 或 /tf_static 并填充上面的 buffer
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 打印等待信息
    RCLCPP_INFO(get_logger(), "Waiting for TF map -> base_link...");
    // 循环直到 ROS 关闭或变换可用
    while (rclcpp::ok())
    {
        try
        {
            // 尝试查找 map 到 base_link 的变换（TimePointZero 表示最新的变换）
            const auto transform_stamped = tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
            (void)transform_stamped;
            // 如果没有抛出异常，说明变换可用
            RCLCPP_INFO(get_logger(), "TF map -> base_link is available.");
            break;
        }
        catch (const tf2::TransformException &ex)
        {
            // 如果查找失败则短暂休眠后重试
            rclcpp::sleep_for(200ms);
        }
    }
}

// 等待 action server 可用，返回布尔值
bool Nav2Client::wait_for_server()
{
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    // 等待 action server 最多 10 秒，返回是否可用
    const bool is_ready = mActionClient->wait_for_action_server(10s);
    return is_ready;
}

// 在仿真或使用 /use_sim_time 时，等待时间被发布（即时钟非 0）
void Nav2Client::wait_for_time()
{
    // 说明：在仿真（use_sim_time=true）场景，若 /clock 尚未发布，ROS 时间会一直是 0。
    // 这里阻塞等待直到时间变为非 0；如果进程收到退出信号（rclcpp::ok()==false），则提前返回。
    while (true)
    {
        const bool is_ok = rclcpp::ok();
        if (false == is_ok)
        {
            return;
        }

        const rcl_duration_value_t now_ns = this->get_clock()->now().nanoseconds();
        if (0 != now_ns)
        {
            return;
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
    msg.header.frame_id = "map";
    // 时间戳设为当前时间
    msg.header.stamp = this->now();
    // 设置初始位置 x, y
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;

    // 构造四元数并设置为无旋转（roll/pitch/yaw 全为 0）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
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
        mInitialPosePublisher->publish(msg);
        rclcpp::sleep_for(200ms);
    }
}

// 发送导航目标并返回 GoalHandle（或 nullptr 表示失败）
Nav2Client::GoalHandle::SharedPtr Nav2Client::send_goal()
{
    // 构造目标位姿，使用 map 作为参考坐标系
    PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    // 设置目标位置 x, y
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.0;

    // 设置目标朝向（无旋转）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    goal.pose.orientation = tf2::toMsg(q);

    // 构造 NavigateToPose 的 Goal 消息并赋值
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal;

    // 配置发送选项，包括反馈回调
    auto send_goal_options = NavigateClient::SendGoalOptions();
    // 反馈回调：打印剩余距离（feedback->distance_remaining）
    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    { RCLCPP_INFO(get_logger(), "Remaining distance: %.2f", feedback->distance_remaining); };

    // 异步发送目标，返回一个 future（用于拿到 goal handle）
    auto goal_handle_future = mActionClient->async_send_goal(goal_msg, send_goal_options);

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
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        return nullptr;
    }

    // 目标被接受，返回 GoalHandle，后续可以用来获取结果或取消
    RCLCPP_INFO(get_logger(), "Goal accepted, waiting for result...");
    return goal_handle;
}
