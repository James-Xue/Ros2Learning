#include "ros2_learning_actions/count_action_server.hpp"

/**
 * @file count_action_server.cpp
 * @brief CountActionServer 实现：目标校验、计数循环、Feedback 发布、取消处理。
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
 */
CountActionServer::CountActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("count_action_server", options)
{
    // 创建 action server，绑定 handle_goal / handle_cancel / handle_accepted
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
 */
CountActionServer::~CountActionServer()
{
    // join 而非 detach：execute() 持有 goal_handle，
    // detach 后 goal_handle 可能访问已销毁的 executor 资源
    std::lock_guard<std::mutex> lock(thread_mutex_);
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
    // 校验 target 必须大于 0
    if (goal->target <= 0) {
        RCLCPP_WARN(get_logger(),
            "[CountActionServer] REJECT: target=%d <= 0", goal->target);
        return rclcpp_action::GoalResponse::REJECT;
    }
    // 校验 period_sec 必须是有限正值且不超过上限（防御 NaN/Inf/极大值）
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
    RCLCPP_INFO(get_logger(), "[CountActionServer] 接受取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 处理已接受目标：join 旧线程后启动新执行线程。
 * @param[in] goal_handle 已接受的目标句柄。
 */
void CountActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleCountTask> goal_handle)
{
    // handle_accepted 是非阻塞回调，不能在此执行耗时操作。
    // mutex 保护 execute_thread_：多线程 executor 下回调可能并发到达
    std::lock_guard<std::mutex> lock(thread_mutex_);
    // 先 join 前一个执行线程（单目标模式，不支持并发 goal）。
    // 注意：此处 join 会阻塞直到旧任务完成。对学习包而言可以接受；
    // 生产代码应改用 goal preemption 或 goal queue。
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    // 启动独立线程执行计数循环
    execute_thread_ = std::thread(
        &CountActionServer::execute, this, goal_handle);
}

/**
 * @brief 执行计数循环：递增计数、发布 Feedback、检查取消。
 * @param[in] goal_handle 目标句柄。
 */
void CountActionServer::execute(
    const std::shared_ptr<GoalHandleCountTask> goal_handle)
{
    const auto start_time = std::chrono::steady_clock::now();
    const auto & goal = *goal_handle->get_goal();

    RCLCPP_INFO(get_logger(),
        "[CountActionServer] 开始执行：target=%d, period=%.3fs",
        goal.target, static_cast<double>(goal.period_sec));

    // 计数循环：从 1 到 target，每次等待 period_sec 秒。
    // 外层 try-catch 防御节点/executor 在执行过程中被销毁的场景
    // （例如测试 TearDown 先于 execute 完成）。
    try {
        for (int32_t current = 1; current <= goal.target && rclcpp::ok(); ++current) {
            // 等待前检查取消（避免额外等待一个 period）
            if (goal_handle->is_canceling()) {
                const float elapsed = compute_elapsed_sec(start_time);
                RCLCPP_INFO(get_logger(),
                    "[CountActionServer] 等待前检测到取消，current=%d",
                    current - 1);
                cancel_with_result(goal_handle, current - 1, elapsed);
                return;
            }

            // 等待一个 period（模拟每次计数的耗时）
            std::this_thread::sleep_for(
                std::chrono::duration<float>(goal.period_sec));

            // 等待后再次检查取消
            if (goal_handle->is_canceling()) {
                const float elapsed = compute_elapsed_sec(start_time);
                RCLCPP_INFO(get_logger(),
                    "[CountActionServer] 等待后检测到取消，current=%d", current);
                cancel_with_result(goal_handle, current, elapsed);
                return;
            }

            // 发布本次计数的 Feedback
            publish_feedback(goal_handle, current, goal.target);
        }

        // 正常完成
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
 */
void CountActionServer::publish_feedback(
    const std::shared_ptr<GoalHandleCountTask> & goal_handle,
    int32_t current, int32_t target)
{
    auto feedback = std::make_shared<CountTask::Feedback>();
    feedback->current = current;
    // target 在 handle_goal 已校验 > 0，不会除零
    feedback->percent =
        static_cast<float>(current) / static_cast<float>(target) * 100.0f;

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
    goal_handle->succeed(result);
}

/**
 * @brief 返回取消 Result。
 * @param[in] goal_handle 目标句柄。
 * @param[in] final_count 被取消时已达到的计数值。
 * @param[in] elapsed_sec 耗时。
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
