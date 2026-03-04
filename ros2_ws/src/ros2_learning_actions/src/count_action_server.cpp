#include "ros2_learning_actions/count_action_server.hpp"

/**
 * @file count_action_server.cpp
 * @brief CountActionServer 具体实现：负责 Goal 校验、Feedback 推送与 Result 收敛。
 *
 * 详细说明：本文件实现 ROS 2 Action Server 的 Goal/Result/Feedback 三段式通信。
 * 客户端先发送 Goal（目标计数与周期），服务端在 `handle_goal` 判定是否接收；接收后
 * 在 `execute` 周期发布 Feedback（当前计数与百分比）；任务完成或取消时返回 Result。
 * 线程模型上，Server 将耗时执行放入 `std::thread`，避免阻塞 executor 的回调线程，
 * 并用 `std::mutex + std::lock_guard` 保护 `execute_thread_` 生命周期。`handle_goal`、
 * `handle_cancel`、`handle_accepted` 分别在收到目标请求、收到取消请求、目标被框架
 * 接受后触发；`execute` 由 `handle_accepted` 启动，承担真正业务循环。
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_learning_actions
{

/**
 * @brief 构造 CountActionServer，创建 action server 并绑定三个回调。
 * @param[in] options ROS 2 节点选项。
 * @return 无（构造函数）。
 */
CountActionServer::CountActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("count_action_server", options)
{
    // WHY: create_server 会向中间件注册 action 名称与协议端点（goal/cancel/result/feedback/status），
    // 并把三个回调挂接到 rclcpp_action 的状态机入口，后续由 executor 在事件到来时调度执行。
    action_server_ = rclcpp_action::create_server<CountTask>(
        this,
        "count_task",
        std::bind(&CountActionServer::handle_goal, this, _1, _2),
        std::bind(&CountActionServer::handle_cancel, this, _1),
        std::bind(&CountActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "[CountActionServer] 已启动，action: /count_task");
}

/**
 * @brief 析构：join 执行线程以确保安全退出。
 * @return 无（析构函数）。
 */
CountActionServer::~CountActionServer()
{
    // WHY: 使用 RAII 锁（lock_guard）保证异常/早返回时也能自动解锁，避免死锁风险。
    // 这里保护的是 execute_thread_ 这个共享线程对象，防止与 handle_accepted 并发访问冲突。
    std::lock_guard<std::mutex> lock(thread_mutex_);
    // WHY: 必须 join 而非 detach。execute() 仍可能访问 goal_handle 和 logger，
    // 若节点析构后线程仍运行，会触发悬垂访问或未定义行为。
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
}

/**
 * @brief 处理目标请求：校验 target > 0 且 period_sec > 0。
 * @param[in] uuid 目标唯一标识（框架生成）。
 * @param[in] goal 客户端提交的目标。
 * @return ACCEPT_AND_EXECUTE 或 REJECT。
 */
rclcpp_action::GoalResponse CountActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const CountTask::Goal> goal)
{
    // WHY: 提前拒绝非法 Goal，避免把无效输入带入执行线程，减少后续状态分支复杂度。
    if (goal->target <= 0) {
        RCLCPP_WARN(get_logger(),
            "[CountActionServer] REJECT: target=%d <= 0", goal->target);
        return rclcpp_action::GoalResponse::REJECT;
    }
    // WHY: 限定 period 范围，防御 NaN/Inf/极端值导致 sleep 行为不可预测或任务卡死。
    constexpr float kMaxPeriodSec = 3600.0f;  // 1 小时上限
    if (!std::isfinite(goal->period_sec) || goal->period_sec <= 0.0f ||
        goal->period_sec > kMaxPeriodSec)
    {
        RCLCPP_WARN(get_logger(),
            "[CountActionServer] REJECT: period_sec=%.3f 不在 (0, %.0f] 范围内",
            static_cast<double>(goal->period_sec),
            static_cast<double>(kMaxPeriodSec));
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(),
        "[CountActionServer] ACCEPT: target=%d, period_sec=%.3f",
        goal->target, static_cast<double>(goal->period_sec));
    // WHY: 返回 ACCEPT_AND_EXECUTE 表示让框架自动推进到 accepted，并触发 handle_accepted。
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief 处理取消请求：无条件接受。
 * @param[in] goal_handle 目标句柄。
 * @return 始终返回 ACCEPT。
 */
rclcpp_action::CancelResponse CountActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleCountTask> /*goal_handle*/)
{
    // WHY: 统一接受取消请求，把“是否能安全中断”决策放到 execute 内部检查点处理。
    RCLCPP_INFO(get_logger(), "[CountActionServer] 接受取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 处理已接受目标：join 旧线程后启动新执行线程。
 * @param[in] goal_handle 已接受的目标句柄。
 * @return 无。
 */
void CountActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleCountTask> goal_handle)
{
    // WHY: handle_accepted 运行在 executor 回调上下文，若直接执行长循环会阻塞其他事件。
    // 因此仅在此做线程调度，把耗时逻辑下沉到 execute 线程。
    // 同时加锁保护 execute_thread_，避免多线程 executor 下并发修改同一线程对象。
    std::lock_guard<std::mutex> lock(thread_mutex_);
    // WHY: 当前实现采用单 Goal 串行模型。若已有旧线程，先 join 保证状态收敛后再接新 Goal。
    // 这保证了线程生命周期清晰，但会引入等待；学习示例可接受。
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    // WHY: 独立线程执行 execute，避免阻塞 action 回调线程，保持节点响应性。
    execute_thread_ = std::thread(
        &CountActionServer::execute, this, goal_handle);
}

/**
 * @brief 执行计数循环：递增计数、发布 Feedback、检查取消。
 * @param[in] goal_handle 目标句柄。
 * @return 无。
 */
void CountActionServer::execute(
    const std::shared_ptr<GoalHandleCountTask> goal_handle)
{
    const auto start_time = std::chrono::steady_clock::now();
    // WHY: get_goal 读取的是框架保存的 Goal 快照；后续只读访问，避免竞态。
    const auto & goal = *goal_handle->get_goal();

    RCLCPP_INFO(get_logger(),
        "[CountActionServer] 开始执行：target=%d, period=%.3fs",
        goal.target, static_cast<double>(goal.period_sec));

    // WHY: 外层 try-catch 用于兜底线程异常，确保后台线程不会因未捕获异常直接终止进程。
    try {
        for (int32_t current = 1; current <= goal.target && rclcpp::ok(); ++current) {
            // WHY: 先检查取消，避免用户已取消却还要多等待一个周期。
            if (goal_handle->is_canceling()) {
                const float elapsed = compute_elapsed_sec(start_time);
                RCLCPP_INFO(get_logger(),
                    "[CountActionServer] 等待前检测到取消，current=%d",
                    current - 1);
                cancel_with_result(goal_handle, current - 1, elapsed);
                return;
            }

            // WHY: sleep_for 模拟任务子步骤耗时，让 Feedback 进度可观测。
            std::this_thread::sleep_for(
                std::chrono::duration<float>(goal.period_sec));

            // WHY: 等待期间可能收到 cancel，因此等待后再检查一次确保快速响应。
            if (goal_handle->is_canceling()) {
                const float elapsed = compute_elapsed_sec(start_time);
                RCLCPP_INFO(get_logger(),
                    "[CountActionServer] 等待后检测到取消，current=%d", current);
                cancel_with_result(goal_handle, current, elapsed);
                return;
            }

            // WHY: 每次迭代发布 Feedback，客户端可实时感知长任务进度而非“黑盒等待”。
            publish_feedback(goal_handle, current, goal.target);
        }

        // WHY: 仅在 ROS 上下文仍有效时上报成功，避免关闭期发送无效通信。
        if (rclcpp::ok()) {
            const float elapsed = compute_elapsed_sec(start_time);
            succeed_with_result(goal_handle, goal.target, elapsed);
        }
    } catch (const std::exception & e) {
        // 节点/executor 已被销毁或其他异常，安全退出线程
        RCLCPP_WARN(get_logger(),
            "[CountActionServer] execute 异常退出: %s", e.what());
    }
}

/**
 * @brief 发布一次 Feedback 消息。
 * @param[in] goal_handle 目标句柄。
 * @param[in] current     当前计数值。
 * @param[in] target      目标计数值。
 * @return 无。
 */
void CountActionServer::publish_feedback(
    const std::shared_ptr<GoalHandleCountTask> & goal_handle,
    int32_t current, int32_t target)
{
    auto feedback = std::make_shared<CountTask::Feedback>();
    feedback->current = current;
    // WHY: target 在 handle_goal 已校验 > 0，可安全计算百分比且避免除零。
    feedback->percent =
        static_cast<float>(current) / static_cast<float>(target) * 100.0f;

    // WHY: publish_feedback 会通过 action feedback 通道异步发送中间状态，
    // 不改变 goal 最终状态，仅用于进度可视化。
    goal_handle->publish_feedback(feedback);

    RCLCPP_DEBUG(get_logger(),
        "[CountActionServer] Feedback: current=%d, percent=%.1f%%",
        current, static_cast<double>(feedback->percent));
}

/**
 * @brief 返回成功 Result。
 * @param[in] goal_handle 目标句柄。
 * @param[in] final_count 最终计数值。
 * @param[in] elapsed_sec 任务总耗时。
 * @return 无。
 */
void CountActionServer::succeed_with_result(
    const std::shared_ptr<GoalHandleCountTask> & goal_handle,
    int32_t final_count, float elapsed_sec)
{
    auto result = std::make_shared<CountTask::Result>();
    result->final_count = final_count;
    result->elapsed_sec = elapsed_sec;
    result->succeeded = true;

    RCLCPP_INFO(get_logger(),
        "[CountActionServer] 成功完成：final_count=%d, elapsed=%.3fs",
        final_count, static_cast<double>(elapsed_sec));
    // WHY: succeed 会把 goal 终态写为 SUCCEEDED，并触发客户端 result 回调。
    goal_handle->succeed(result);
}

/**
 * @brief 返回取消 Result。
 * @param[in] goal_handle 目标句柄。
 * @param[in] final_count 被取消时已达到的计数值。
 * @param[in] elapsed_sec 耗时。
 * @return 无。
 */
void CountActionServer::cancel_with_result(
    const std::shared_ptr<GoalHandleCountTask> & goal_handle,
    int32_t final_count, float elapsed_sec)
{
    auto result = std::make_shared<CountTask::Result>();
    result->final_count = final_count;
    result->elapsed_sec = elapsed_sec;
    result->succeeded = false;

    RCLCPP_INFO(get_logger(),
        "[CountActionServer] 取消完成：final_count=%d, elapsed=%.3fs",
        final_count, static_cast<double>(elapsed_sec));
    // WHY: canceled 会把 goal 终态写为 CANCELED，客户端据此区分正常完成与中断。
    goal_handle->canceled(result);
}

/**
 * @brief 计算自给定时刻起的经过时间（秒）。
 * @param[in] start_time 计时起始时刻。
 * @return 经过时间（float，秒）。
 */
float CountActionServer::compute_elapsed_sec(
    const std::chrono::steady_clock::time_point & start_time)
{
    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    return std::chrono::duration<float>(elapsed).count();
}

}  // namespace ros2_learning_actions

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_actions::CountActionServer)
