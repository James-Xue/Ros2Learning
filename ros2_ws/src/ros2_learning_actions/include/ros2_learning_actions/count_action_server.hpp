#pragma once

/**
 * @file count_action_server.hpp
 * @brief CountActionServer 节点声明——演示 rclcpp_action Server 完整实现。
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_learning_custom_interfaces/action/count_task.hpp>

namespace ros2_learning_actions
{

/**
 * @class CountActionServer
 * @brief 基于 CountTask.action 的计数动作服务端。
 *
 * 功能设计：
 *   - handle_goal    : 校验 target > 0 且 period_sec > 0，否则 REJECT
 *   - handle_cancel  : 无条件接受取消请求
 *   - handle_accepted: 在独立 std::thread 中启动计数循环
 *   - execute        : 循环递增计数，每次等待 period_sec 秒，发布 Feedback，
 *                      检查 is_canceling()；完成后调用 succeed() 或 canceled()
 *
 * 线程安全说明：
 *   execute_thread_ 在 handle_accepted 中启动，持有 goal_handle 的 shared_ptr，
 *   与 ROS 回调线程完全解耦。通过 goal_handle->is_canceling() 安全检查取消状态。
 *   不支持并发 goal——handle_accepted 会先 join 前一个执行线程。
 *
 * 注册为 rclcpp_components 插件，可独立运行或放入 ComposableNodeContainer。
 */
class CountActionServer : public rclcpp::Node
{
public:
    /// Action 接口类型别名。
    using CountTask = ros2_learning_custom_interfaces::action::CountTask;
    /// Server 端 GoalHandle 类型别名。
    using GoalHandleCountTask = rclcpp_action::ServerGoalHandle<CountTask>;

    /**
     * @brief 构造 CountActionServer 并创建 action server。
     * @param[in] options ROS 2 节点选项。
     */
    explicit CountActionServer(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 析构，确保执行线程安全退出。
     */
    ~CountActionServer() override;

private:
    /**
     * @brief 处理目标请求回调，校验参数合法性。
     * @param[in] uuid 目标唯一标识（框架生成）。
     * @param[in] goal 客户端提交的目标（target, period_sec）。
     * @return ACCEPT_AND_EXECUTE 或 REJECT。
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CountTask::Goal> goal);

    /**
     * @brief 处理取消请求回调，无条件接受。
     * @param[in] goal_handle 目标句柄。
     * @return 始终返回 ACCEPT。
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCountTask> goal_handle);

    /**
     * @brief 处理已接受目标回调，启动执行线程。
     * @param[in] goal_handle 已接受的目标句柄。
     */
    void handle_accepted(
        const std::shared_ptr<GoalHandleCountTask> goal_handle);

    /**
     * @brief 在独立线程中执行计数循环，发布 Feedback，完成后报告 Result。
     * @param[in] goal_handle 目标句柄。
     */
    void execute(const std::shared_ptr<GoalHandleCountTask> goal_handle);

    /**
     * @brief 构造并发布一次 Feedback 消息。
     * @param[in] goal_handle 目标句柄。
     * @param[in] current     当前计数值。
     * @param[in] target      目标计数值（用于计算 percent）。
     */
    void publish_feedback(
        const std::shared_ptr<GoalHandleCountTask> & goal_handle,
        int32_t current,
        int32_t target);

    /**
     * @brief 构造成功的 Result 并通过 goal_handle 返回。
     * @param[in] goal_handle 目标句柄。
     * @param[in] final_count 最终计数值。
     * @param[in] elapsed_sec 任务总耗时（秒）。
     */
    void succeed_with_result(
        const std::shared_ptr<GoalHandleCountTask> & goal_handle,
        int32_t final_count,
        float elapsed_sec);

    /**
     * @brief 构造取消的 Result 并通过 goal_handle 返回。
     * @param[in] goal_handle 目标句柄。
     * @param[in] final_count 被取消时已达到的计数值。
     * @param[in] elapsed_sec 从开始到取消的耗时（秒）。
     */
    void cancel_with_result(
        const std::shared_ptr<GoalHandleCountTask> & goal_handle,
        int32_t final_count,
        float elapsed_sec);

    /**
     * @brief 计算自给定时刻起的经过时间。
     * @param[in] start_time 计时起始时刻。
     * @return 经过时间（float，秒）。
     */
    static float compute_elapsed_sec(
        const std::chrono::steady_clock::time_point & start_time);

    rclcpp_action::Server<CountTask>::SharedPtr action_server_;  ///< action server 实例。
    std::thread execute_thread_;  ///< 执行计数循环的后台线程。
    std::mutex thread_mutex_;  ///< 保护 execute_thread_ 的互斥锁。
};

}  // namespace ros2_learning_actions
