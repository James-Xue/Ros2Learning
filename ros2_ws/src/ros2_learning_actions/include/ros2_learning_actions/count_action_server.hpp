#pragma once

/**
 * @file count_action_server.hpp
 * @brief CountActionServer 声明：定义 Action Server 的回调接口与线程协作边界。
 *
 * 详细说明：该头文件描述 ROS 2 Action Server 在 Goal/Feedback/Result 三段式协议中的
 * 角色。Goal 阶段由 `handle_goal` 审核并决定接受或拒绝；执行阶段由 `execute` 迭代发布
 * Feedback；结束阶段通过 `succeed_with_result`/`cancel_with_result` 回传 Result。
 * 线程模型上，`handle_accepted` 仅负责调度，将耗时任务放入 `std::thread`；通过
 * `std::mutex` 串行保护 `execute_thread_`，避免并发回调下线程对象竞争。
 * 在 rclcpp_action 框架中，`handle_goal` 在收到 goal request 时触发，`handle_cancel`
 * 在收到 cancel request 时触发，`handle_accepted` 在 goal 状态进入 accepted 后触发，
 * `execute` 为业务执行体，不直接由 executor 长时占用回调线程。
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
 * @brief 基于 `rclcpp::Node` 的计数 Action 服务端，负责受理目标并执行计数任务。
 *
 * 设计说明：
 * - 继承 `rclcpp::Node`，因为 Action Server 需要依托 ROS 2 节点完成实体创建、日志输出、
 *   参数/时钟/执行器上下文接入；该继承也是 rclcpp_components 组件化加载的标准入口。
 * - 执行流程是一个简化状态机：`handle_goal`(校验) -> `handle_accepted`(调度线程) ->
 *   `execute`(循环发布 Feedback) -> `succeed_with_result/cancel_with_result`(终态回传)。
 * - 线程安全上，`thread_mutex_` 仅保护 `execute_thread_` 的创建/join 生命周期；
 *   目标取消状态由 `goal_handle->is_canceling()` 从框架状态机读取，不由本类自行加锁。
 *
 * 生命周期说明：
 * - 该类通常由组件容器创建并托管，析构时会尝试 join 执行线程，确保退出期无后台悬挂任务。
 * - 当前实现为单 Goal 串行模型（新 Goal 到来前会等待旧线程收敛），用于教学可观测性优先。
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
     * @return 无（构造函数）。
     */
    explicit CountActionServer(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 析构，确保执行线程安全退出。
     * @return 无（析构函数）。
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
     * @return 无。
     */
    void handle_accepted(
        const std::shared_ptr<GoalHandleCountTask> goal_handle);

    /**
     * @brief 在独立线程中执行计数循环，发布 Feedback，完成后报告 Result。
     * @param[in] goal_handle 目标句柄。
     * @return 无。
     */
    void execute(const std::shared_ptr<GoalHandleCountTask> goal_handle);

    /**
     * @brief 构造并发布一次 Feedback 消息。
     * @param[in] goal_handle 目标句柄。
     * @param[in] current     当前计数值。
     * @param[in] target      目标计数值（用于计算 percent）。
     * @return 无。
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
     * @return 无。
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
     * @return 无。
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

    rclcpp_action::Server<CountTask>::SharedPtr action_server_;  ///< Action Server 句柄，节点存活期间常驻，用于接入 rclcpp_action 状态机。
    std::thread execute_thread_;  ///< 后台执行线程，仅在有已接受 Goal 时有效，析构前必须 join 回收。
    std::mutex thread_mutex_;  ///< 线程互斥锁，生命周期同节点，用于保护 execute_thread_ 的 join/create 临界区。
};

}  // namespace ros2_learning_actions
