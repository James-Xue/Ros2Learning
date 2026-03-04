#include "ros2_learning_actions/count_action_client.hpp"

/**
 * @file count_action_client.cpp
 * @brief CountActionClient 实现：参数化目标发送、Feedback 打印、超时取消演示。
 */

#include <chrono>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_learning_actions
{

/**
 * @brief 构造 CountActionClient，声明参数并在 executor 就绪后发送 goal。
 * @param[in] options ROS 2 节点选项。
 */
CountActionClient::CountActionClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("count_action_client", options)
{
    declare_parameters();

    // 创建 action client，名称与 server 端一致
    action_client_ = rclcpp_action::create_client<CountTask>(
        this, "count_task");

    // one-shot timer：等 executor 启动后再发送 goal
    // 不能在构造函数中直接 wait_for_action_server，因为 executor 可能未启动
    init_timer_ = create_wall_timer(100ms, [this]() {
        init_timer_->cancel();  // one-shot：首次触发后取消
        send_goal();
    });
}

/**
 * @brief 声明客户端参数。
 */
void CountActionClient::declare_parameters()
{
    declare_parameter("target", 10);
    declare_parameter("period_sec", 0.5);
    declare_parameter("cancel_after_sec", -1.0);
}

/**
 * @brief 等待 server 就绪并发送 goal。
 */
void CountActionClient::send_goal()
{
    if (!action_client_->wait_for_action_server(2s)) {
        RCLCPP_ERROR(get_logger(),
            "[CountActionClient] Action server 不可用，放弃");
        return;
    }

    // 从参数读取 goal 配置
    CountTask::Goal goal;
    goal.target = static_cast<int32_t>(get_parameter("target").as_int());
    goal.period_sec = static_cast<float>(get_parameter("period_sec").as_double());

    RCLCPP_INFO(get_logger(),
        "[CountActionClient] 发送 goal: target=%d, period_sec=%.3f",
        goal.target, static_cast<double>(goal.period_sec));

    // 配置三个回调
    auto send_options = rclcpp_action::Client<CountTask>::SendGoalOptions();
    send_options.goal_response_callback =
        std::bind(&CountActionClient::goal_response_callback, this, _1);
    send_options.feedback_callback =
        std::bind(&CountActionClient::feedback_callback, this, _1, _2);
    send_options.result_callback =
        std::bind(&CountActionClient::result_callback, this, _1);

    action_client_->async_send_goal(goal, send_options);
}

/**
 * @brief goal 被接受或拒绝时的回调。
 * @param[in] goal_handle 若非 nullptr 表示被接受；nullptr 表示被 server REJECT。
 */
void CountActionClient::goal_response_callback(
    const GoalHandleCountTask::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(),
            "[CountActionClient] goal 被拒绝！检查 target 和 period_sec");
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[CountActionClient] goal 已被接受，等待执行中...");

    // 保存 handle 供取消使用
    goal_handle_ = goal_handle;

    // 若 cancel_after_sec > 0，调度超时取消
    const double cancel_after = get_parameter("cancel_after_sec").as_double();
    if (cancel_after > 0.0) {
        RCLCPP_INFO(get_logger(),
            "[CountActionClient] 将在 %.1f 秒后发送取消请求", cancel_after);
        schedule_cancel();
    }
}

/**
 * @brief 收到 Feedback 时的回调。
 * @param[in] goal_handle goal 句柄（未使用）。
 * @param[in] feedback    Server 发来的 Feedback。
 */
void CountActionClient::feedback_callback(
    GoalHandleCountTask::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const CountTask::Feedback> feedback)
{
    RCLCPP_INFO(get_logger(),
        "[CountActionClient] 进度: current=%d (%.1f%%)",
        feedback->current, static_cast<double>(feedback->percent));
}

/**
 * @brief 收到最终 Result 时的回调。
 * @param[in] result 封装结果。
 */
void CountActionClient::result_callback(
    const GoalHandleCountTask::WrappedResult & result)
{
    // 清理资源
    cancel_timer_.reset();
    goal_handle_.reset();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(),
                "[CountActionClient] 成功！final_count=%d, elapsed=%.3fs",
                result.result->final_count,
                static_cast<double>(result.result->elapsed_sec));
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(),
                "[CountActionClient] 已取消。final_count=%d, elapsed=%.3fs",
                result.result->final_count,
                static_cast<double>(result.result->elapsed_sec));
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(),
                "[CountActionClient] 被中止（ABORTED）");
            break;
        default:
            RCLCPP_ERROR(get_logger(),
                "[CountActionClient] 未知结果代码");
    }
}

/**
 * @brief 调度取消超时定时器。
 */
void CountActionClient::schedule_cancel()
{
    const double cancel_after = get_parameter("cancel_after_sec").as_double();
    const auto delay = std::chrono::duration<double>(cancel_after);

    cancel_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
        [this]() {
            cancel_timer_->cancel();  // one-shot
            if (goal_handle_) {
                RCLCPP_WARN(get_logger(),
                    "[CountActionClient] 超时，发送取消请求...");
                action_client_->async_cancel_goal(goal_handle_);
            }
        });
}

}  // namespace ros2_learning_actions

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_actions::CountActionClient)
