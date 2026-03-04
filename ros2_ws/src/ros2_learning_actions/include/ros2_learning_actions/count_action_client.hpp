#pragma once

/**
 * @file count_action_client.hpp
 * @brief CountActionClient 节点声明——演示 rclcpp_action Client 完整实现。
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_learning_custom_interfaces/action/count_task.hpp>

namespace ros2_learning_actions
{

/**
 * @class CountActionClient
 * @brief 基于 CountTask.action 的计数动作客户端。
 *
 * 功能设计：
 *   - 启动后等待 action server 就绪
 *   - 从 ROS 参数读取 target、period_sec、cancel_after_sec
 *   - 设置 goal_response_callback、feedback_callback、result_callback
 *   - 若 cancel_after_sec > 0，在对应延迟后发送取消请求
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
     */
    explicit CountActionClient(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
    /**
     * @brief 声明客户端节点使用的全部参数。
     */
    void declare_parameters();

    /**
     * @brief 等待 action server 就绪并发送 goal。
     */
    void send_goal();

    /**
     * @brief goal 被接受或拒绝时的回调。
     * @param[in] goal_handle 若非 nullptr 表示被接受；nullptr 表示被拒绝。
     */
    void goal_response_callback(
        const GoalHandleCountTask::SharedPtr & goal_handle);

    /**
     * @brief 收到 Feedback 时的回调，打印当前进度。
     * @param[in] goal_handle goal 句柄（未使用）。
     * @param[in] feedback    Server 发来的 Feedback 消息。
     */
    void feedback_callback(
        GoalHandleCountTask::SharedPtr goal_handle,
        const std::shared_ptr<const CountTask::Feedback> feedback);

    /**
     * @brief 收到最终 Result 时的回调，打印结果并清理资源。
     * @param[in] result 包含 ResultCode 和 Result 字段的封装结果。
     */
    void result_callback(const GoalHandleCountTask::WrappedResult & result);

    /**
     * @brief 在 cancel_after_sec 到期后发送取消请求。
     */
    void schedule_cancel();

    rclcpp_action::Client<CountTask>::SharedPtr action_client_;  ///< action client 实例。
    GoalHandleCountTask::SharedPtr goal_handle_;  ///< 已接受的 goal 句柄（用于取消）。
    rclcpp::TimerBase::SharedPtr init_timer_;  ///< 延迟初始化 timer（one-shot）。
    rclcpp::TimerBase::SharedPtr cancel_timer_;  ///< 取消超时定时器（可选）。
};

}  // namespace ros2_learning_actions
