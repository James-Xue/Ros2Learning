#pragma once

/**
 * @file count_action_client.hpp
 * @brief CountActionClient 声明：定义 Action Client 的异步交互接口与回调契约。
 *
 * 详细说明：该头文件描述客户端如何参与 ROS 2 Action 的 Goal/Feedback/Result 三段式通信。
 * 客户端通过 `send_goal` 发起 Goal；执行期间由 `feedback_callback` 接收进度；最终由
 * `result_callback` 收敛状态。线程模型上，客户端回调和定时器可能并发触发，工程实践中
 * 常配合 MultiThreadedExecutor 提升响应能力；本类本身不阻塞等待，采用纯异步回调流。
 * 在 rclcpp_action 框架内，`handle_goal/handle_cancel/handle_accepted/execute` 位于服务端，
 * 客户端通过 goal response、feedback、result 这三类回调观测对应阶段结果。
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_learning_custom_interfaces/action/count_task.hpp>

namespace ros2_learning_actions
{

/**
 * @class CountActionClient
 * @brief 基于 `rclcpp::Node` 的计数 Action 客户端，负责目标下发与结果处理。
 *
 * 设计说明：
 * - 继承 `rclcpp::Node`，用于复用 ROS 2 的参数系统、日志系统、定时器与执行器上下文。
 * - 执行流程是异步状态流：构造 -> `declare_parameters` -> `send_goal` ->
 *   `goal_response_callback` -> 多次 `feedback_callback` -> `result_callback` 收敛。
 * - 线程安全方面：本类未引入显式 `std::mutex`，因为共享状态仅为 `goal_handle_` 与 timer 指针，
 *   由回调顺序和生命周期管理控制访问；若扩展为复杂共享数据，应显式加锁保护临界区。
 *
 * 参数：
 *   target           (int,   default=10)   — 计数目标
 *   period_sec       (double, default=0.5) — 每次计数的等待时间（秒）
 *   cancel_after_sec (double, default=-1.0)— 正值时在该时间后发送取消；负值不取消
 *
 * 注册为 rclcpp_components 插件，可独立运行或放入 ComposableNodeContainer。
 */
class CountActionClient : public rclcpp::Node
{
public:
    /// Action 接口类型别名。
    using CountTask = ros2_learning_custom_interfaces::action::CountTask;
    /// Client 端 GoalHandle 类型别名。
    using GoalHandleCountTask = rclcpp_action::ClientGoalHandle<CountTask>;

    /**
     * @brief 构造 CountActionClient，声明参数并在 executor 就绪后发送 goal。
     * @param[in] options ROS 2 节点选项。
     * @return 无（构造函数）。
     */
    explicit CountActionClient(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    /**
     * @brief 声明客户端节点使用的全部参数。
     * @return 无。
     */
    void declare_parameters();

    /**
     * @brief 等待 action server 就绪并发送 goal。
     * @return 无。
     */
    void send_goal();

    /**
     * @brief goal 被接受或拒绝时的回调。
     * @param[in] goal_handle 若非 nullptr 表示被接受；nullptr 表示被拒绝。
     * @return 无。
     */
    void goal_response_callback(
        const GoalHandleCountTask::SharedPtr & goal_handle);

    /**
     * @brief 收到 Feedback 时的回调，打印当前进度。
     * @param[in] goal_handle goal 句柄（未使用）。
     * @param[in] feedback    Server 发来的 Feedback 消息。
     * @return 无。
     */
    void feedback_callback(
        GoalHandleCountTask::SharedPtr goal_handle,
        const std::shared_ptr<const CountTask::Feedback> feedback);

    /**
     * @brief 收到最终 Result 时的回调，打印结果并清理资源。
     * @param[in] result 包含 ResultCode 和 Result 字段的封装结果。
     * @return 无。
     */
    void result_callback(const GoalHandleCountTask::WrappedResult & result);

    /**
     * @brief 在 cancel_after_sec 到期后发送取消请求。
     * @return 无。
     */
    void schedule_cancel();

    rclcpp_action::Client<CountTask>::SharedPtr action_client_;  ///< Action Client 句柄，节点生命周期内常驻，用于异步发送 goal/cancel。
    GoalHandleCountTask::SharedPtr goal_handle_;  ///< 当前已接受 Goal 的句柄，仅在活跃任务期间有效，用于后续取消请求。
    rclcpp::TimerBase::SharedPtr init_timer_;  ///< 启动期 one-shot 定时器，仅用于延迟触发首次 send_goal，触发后即取消。
    rclcpp::TimerBase::SharedPtr cancel_timer_;  ///< 可选取消定时器，仅当 `cancel_after_sec > 0` 时创建并在 Result 到达后释放。
};

}  // namespace ros2_learning_actions
