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
     * @param[in] options ROS 2 节点选项。
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
        std::string name;  ///< 被管理节点名（例如 sensor_node）。
        ChangeStateClient::SharedPtr change_state_client;  ///< /<node>/change_state 客户端。
        GetStateClient::SharedPtr get_state_client;  ///< /<node>/get_state 客户端。
    };

    /**
     * @brief 执行启动序列：等待服务、configure 全部节点、延迟后 activate 全部节点。
     */
    void startup_sequence();

    /**
     * @brief 声明参数并读取受管节点列表。
     * @param[out] node_names 读取到的受管节点名称列表。
     */
    void load_managed_node_names(std::vector<std::string> & node_names);

    /**
     * @brief 为每个受管节点创建 lifecycle 服务客户端。
     * @param[in] node_names 受管节点名称列表。
     */
    void initialize_managed_nodes(const std::vector<std::string> & node_names);

    /**
     * @brief 配置服务回调组并绑定到专用执行器。
     */
    void setup_service_executor();

    /**
     * @brief 启动服务执行器线程与启动序列线程。
     */
    void start_background_threads();

    /**
     * @brief 等待所有被管理节点生命周期服务就绪。
     * @param[in] timeout 每个节点服务等待超时时间。
     * @return 全部节点服务就绪返回 true，否则返回 false。
     */
    bool wait_for_all_services(std::chrono::seconds timeout);

    /**
     * @brief 顺序配置所有处于 Unconfigured 的节点。
     * @return 全部需要配置的节点均成功返回 true，否则返回 false。
     */
    bool configure_all_nodes();

    /**
     * @brief 按参数延迟启动下一阶段状态迁移。
     */
    void wait_transition_delay();

    /**
     * @brief 顺序激活所有处于 Inactive 的节点。
     * @return 全部需要激活的节点均成功返回 true，否则返回 false。
     */
    bool activate_all_nodes();

    /**
     * @brief 执行单节点的 configure 迁移（仅在状态匹配时执行）。
     * @param[in] node 被管理节点描述。
     * @return 迁移成功或无需迁移返回 true，迁移失败返回 false。
     */
    bool configure_node_if_needed(const ManagedNode & node);

    /**
     * @brief 执行单节点的 activate 迁移（仅在状态匹配时执行）。
     * @param[in] node 被管理节点描述。
     * @return 迁移成功或无需迁移返回 true，迁移失败返回 false。
     */
    bool activate_node_if_needed(const ManagedNode & node);

    /**
     * @brief 等待单个被管理节点的生命周期服务就绪。
     * @param[in] node 被管理节点描述。
     * @param[in] timeout 每个服务等待超时时间。
     * @return 两个服务均可用返回 true，否则返回 false。
     */
    bool wait_for_service_ready(const ManagedNode & node, std::chrono::seconds timeout);

    /**
     * @brief 对目标节点发送生命周期状态切换请求。
     * @param[in] node 被管理节点描述。
     * @param[in] transition_id 生命周期 transition ID。
     * @return 服务成功返回 true，否则返回 false。
     */
    bool change_state(const ManagedNode & node, uint8_t transition_id);

    /**
     * @brief 查询目标节点当前生命周期主状态 ID。
     * @param[in] node 被管理节点描述。
     * @return 当前状态 ID；超时返回 PRIMARY_STATE_UNKNOWN。
     */
    uint8_t get_state(const ManagedNode & node);

    std::vector<ManagedNode> managed_nodes_;  ///< 被管理节点集合。
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;  ///< 服务客户端专用回调组。
    rclcpp::executors::SingleThreadedExecutor service_executor_;  ///< 服务回调执行器。
    std::thread service_executor_thread_;  ///< service_executor_ 的后台线程。
    std::thread startup_thread_;  ///< 启动序列线程，不阻塞主 executor。
};

}  // namespace ros2_learning_lifecycle
