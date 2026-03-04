#include "ros2_learning_actions/count_action_client.hpp"

/**
 * @file count_action_client.cpp
 * @brief CountActionClient 具体实现：负责发送 Goal、接收 Feedback，并处理最终 Result。
 *
 * 详细说明：本文件实现 ROS 2 Action Client 的 Goal/Result/Feedback 三段式交互。
 * 节点启动后先创建 Client，再发送 Goal；服务端执行期间通过 Feedback 回调持续上报进度；
 * 最终通过 Result 回调收敛任务状态（成功/取消/中止）。线程模型上，Client 侧通常建议
 * 配合 `MultiThreadedExecutor`，因为目标响应、反馈、结果、定时器取消等回调可能并发到达；
 * 该实现通过回调式异步 API 避免阻塞 executor。`handle_goal/handle_cancel/handle_accepted/
 * execute` 属于服务端回调，本文件作为客户端会通过 `goal_response_callback` 与 `result_callback`
 * 间接观测这些阶段变化。
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
 * @return 无（构造函数）。
 */
CountActionClient::CountActionClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("count_action_client", options)
{
    // WHY: 先声明参数，保证后续 send_goal 时读取到稳定配置。
    declare_parameters();

    // WHY: create_client 会创建 Action 协议所需通信端点，并绑定到当前节点上下文。
    // 仅创建对象不会立即发送请求，真正交互由 async_send_goal/async_cancel_goal 触发。
    action_client_ = rclcpp_action::create_client<CountTask>(
        this, "count_task");

    // WHY: 通过 one-shot timer 延迟发送，避免在构造函数阶段阻塞等待 server。
    // 若此时 executor 尚未 spin，直接 wait_for_action_server 可能引发初始化死等。
    init_timer_ = create_wall_timer(100ms, [this]() {
        init_timer_->cancel();  // one-shot：首次触发后取消
        send_goal();
    });
}

/**
 * @brief 声明客户端参数。
 * @return 无。
 */
void CountActionClient::declare_parameters()
{
    // WHY: 参数化让示例无需改代码即可测试不同 Goal/取消策略，便于学习 action 行为差异。
    declare_parameter("target", 10);
    declare_parameter("period_sec", 0.5);
    declare_parameter("cancel_after_sec", -1.0);
}

/**
 * @brief 等待 server 就绪并发送 goal。
 * @return 无。
 */
void CountActionClient::send_goal()
{
    // WHY: wait_for_action_server 检查远端 server 是否在线，避免盲发 goal 导致无意义重试。
    if (!action_client_->wait_for_action_server(2s)) {
        RCLCPP_ERROR(get_logger(),
            "[CountActionClient] Action server 不可用，放弃");
        return;
    }

    // WHY: 从参数构造 Goal，便于通过 launch/命令行动态调整任务负载。
    CountTask::Goal goal;
    goal.target = static_cast<int32_t>(get_parameter("target").as_int());
    goal.period_sec = static_cast<float>(get_parameter("period_sec").as_double());

    RCLCPP_INFO(get_logger(),
        "[CountActionClient] 发送 goal: target=%d, period_sec=%.3f",
        goal.target, static_cast<double>(goal.period_sec));

    // WHY: Action 是典型异步模型；通过三个回调接入不同生命周期事件。
    // 相比 wait_for_result 阻塞式等待，此处选择回调避免占用 executor 线程。
    auto send_options = rclcpp_action::Client<CountTask>::SendGoalOptions();
    send_options.goal_response_callback =
        std::bind(&CountActionClient::goal_response_callback, this, _1);
    send_options.feedback_callback =
        std::bind(&CountActionClient::feedback_callback, this, _1, _2);
    send_options.result_callback =
        std::bind(&CountActionClient::result_callback, this, _1);

    // WHY: async_send_goal 会异步发送 goal request，并返回 future（内部承载 goal_handle）。
    // 本示例不显式保存该 future，也不使用 std::promise/std::future 手工同步，
    // 而是完全依赖回调驱动，降低阻塞与线程协调复杂度。
    action_client_->async_send_goal(goal, send_options);
}

/**
 * @brief goal 被接受或拒绝时的回调。
 * @param[in] goal_handle 若非 nullptr 表示被接受；nullptr 表示被 server REJECT。
 * @return 无。
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

    // WHY: 保存 goal_handle 是后续发送 cancel 的必要前提。
    goal_handle_ = goal_handle;

    // WHY: 用参数开关取消策略，便于演示“正常完成”与“中途取消”两种终态。
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
 * @return 无。
 */
void CountActionClient::feedback_callback(
    GoalHandleCountTask::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const CountTask::Feedback> feedback)
{
    // WHY: Feedback 只表达中间进度，不代表终态；用于提升长任务可观测性与用户体验。
    RCLCPP_INFO(get_logger(),
        "[CountActionClient] 进度: current=%d (%.1f%%)",
        feedback->current, static_cast<double>(feedback->percent));
}

/**
 * @brief 收到最终 Result 时的回调。
 * @param[in] result 封装结果。
 * @return 无。
 */
void CountActionClient::result_callback(
    const GoalHandleCountTask::WrappedResult & result)
{
    // WHY: Result 到达即表示 Goal 生命周期闭合，及时释放句柄与定时器避免误触发。
    cancel_timer_.reset();
    goal_handle_.reset();

    // WHY: 根据 ResultCode 分支可区分服务端终态（成功/取消/中止），便于上层业务决策。
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
 * @return 无。
 */
void CountActionClient::schedule_cancel()
{
    const double cancel_after = get_parameter("cancel_after_sec").as_double();
    const auto delay = std::chrono::duration<double>(cancel_after);

    // WHY: 使用 one-shot timer 在指定延迟后触发取消，模拟“用户超时中断”场景。
    cancel_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
        [this]() {
            cancel_timer_->cancel();  // one-shot
            if (goal_handle_) {
                RCLCPP_WARN(get_logger(),
                    "[CountActionClient] 超时，发送取消请求...");
                // WHY: async_cancel_goal 会异步发出 cancel request，最终由服务端
                // handle_cancel/is_canceling 处理并在 result_callback 中收敛结果。
                action_client_->async_cancel_goal(goal_handle_);
            }
        });
}

}  // namespace ros2_learning_actions

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_actions::CountActionClient)
