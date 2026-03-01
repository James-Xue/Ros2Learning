#pragma once

/**
 * @file lifecycle_manager_node.hpp
 * @brief Lifecycle 管理器节点声明。
 */

#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

namespace ros2_learning_lifecycle
{

/**
 * @class LifecycleManagerNode
 * @brief 通过标准 lifecycle 服务编排多个 LifecycleNode 的管理器。
 *
 * 工作流程：
 *   1. 等待所有被管理节点的 get_state / change_state 服务就绪
 *   2. 顺序调用 configure_all（依次 configure 所有节点）
 *   3. 延迟 transition_delay_sec 秒后，调用 activate_all（依次 activate 所有节点）
 *
 * 设计要点（重要）：
 *   - 所有服务 Client 使用专用 CallbackGroup，由独立 service_executor_ 线程处理
 *   - startup_sequence 在独立 std::thread 中运行，通过 future.wait_for() 等待响应
 *   - 严禁在回调中调用 rclcpp::spin_until_future_complete(node)：
 *     节点已在 executor 中，再次 add_node 会抛 std::runtime_error
 *
 * 参数：
 *   managed_nodes        (string array, default=["sensor_node","processor_node"])
 *   transition_delay_sec (double,       default=1.0)
 */
class LifecycleManagerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造管理器节点并初始化服务客户端。
     * @param options ROS 2 节点选项。
     */
    explicit LifecycleManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 释放线程与 executor 资源。
     */
    ~LifecycleManagerNode();

private:
    using ChangeState = lifecycle_msgs::srv::ChangeState;
    using GetState = lifecycle_msgs::srv::GetState;
    using ChangeStateClient = rclcpp::Client<ChangeState>;
    using GetStateClient = rclcpp::Client<GetState>;

    struct ManagedNode {
        /// 被管理节点名（例如 sensor_node）。
        std::string name;
        /// 调用 /<node>/change_state 服务的客户端。
        ChangeStateClient::SharedPtr change_state_client;
        /// 调用 /<node>/get_state 服务的客户端。
        GetStateClient::SharedPtr    get_state_client;
    };

    /**
     * @brief 执行启动序列：等待服务、configure 全部节点、延迟后 activate 全部节点。
     */
    void startup_sequence();

    /**
     * @brief 等待单个被管理节点的生命周期服务就绪。
     * @param node 被管理节点描述。
     * @param timeout 每个服务等待超时时间。
     * @return 两个服务均可用返回 true，否则返回 false。
     */
    bool wait_for_service_ready(const ManagedNode & node, std::chrono::seconds timeout);

    /**
     * @brief 对目标节点发送生命周期状态切换请求。
     * @param node 被管理节点描述。
     * @param transition_id 生命周期 transition ID。
     * @return 服务成功返回 true，否则返回 false。
     */
    bool change_state(const ManagedNode & node, uint8_t transition_id);

    /**
     * @brief 查询目标节点当前生命周期主状态 ID。
     * @param node 被管理节点描述。
     * @return 当前状态 ID；超时返回 PRIMARY_STATE_UNKNOWN。
     */
    uint8_t get_state(const ManagedNode & node);

    /// 被管理节点集合。
    std::vector<ManagedNode> managed_nodes_;

    /// 服务客户端专用回调组，避免与主执行器回调争用。
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    /// 仅处理服务客户端回调的单线程执行器。
    rclcpp::executors::SingleThreadedExecutor service_executor_;
    /// service_executor_ 的后台线程。
    std::thread service_executor_thread_;

    /// 启动序列线程，不阻塞主 executor。
    std::thread startup_thread_;
};

}  // namespace ros2_learning_lifecycle
