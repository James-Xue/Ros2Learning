#pragma once

#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

namespace ros2_learning_lifecycle
{

/**
 * LifecycleManagerNode — 普通 rclcpp::Node，通过标准 lifecycle 服务编排多个 LifecycleNode。
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
    explicit LifecycleManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
    ~LifecycleManagerNode();

private:
    struct ManagedNode {
        std::string name;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    get_state_client;
    };

    void startup_sequence();
    bool wait_for_service_ready(const ManagedNode & node, std::chrono::seconds timeout);
    bool change_state(const ManagedNode & node, uint8_t transition_id);
    uint8_t get_state(const ManagedNode & node);

    std::vector<ManagedNode> managed_nodes_;

    // 专用于服务 Client 的 CallbackGroup + 独立 Executor 线程
    // 使用独立 CallbackGroup 避免在已在 executor 中的节点上调用 spin_until_future_complete
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::executors::SingleThreadedExecutor service_executor_;
    std::thread service_executor_thread_;

    // 启动序列在独立线程执行，不阻塞主 executor
    std::thread startup_thread_;
};

}  // namespace ros2_learning_lifecycle
