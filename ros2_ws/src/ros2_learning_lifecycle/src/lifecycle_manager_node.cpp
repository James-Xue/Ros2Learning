#include "ros2_learning_lifecycle/lifecycle_manager_node.hpp"

#include <chrono>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace ros2_learning_lifecycle
{

LifecycleManagerNode::LifecycleManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lifecycle_manager_node", options)
{
    this->declare_parameter<std::vector<std::string>>(
        "managed_nodes", {"sensor_node", "processor_node"});
    this->declare_parameter("transition_delay_sec", 1.0);

    const auto node_names =
        this->get_parameter("managed_nodes").as_string_array();

    // ── 关键设计 ──────────────────────────────────────────────────────
    // 创建专用 CallbackGroup，服务 Client 的响应回调均注册到此 group。
    // 再把此 group（而非整个节点）加入 service_executor_，在独立线程中 spin。
    //
    // 这样做的原因：
    //   rclcpp::spin_until_future_complete(node) 会把节点加入新 executor，
    //   但节点已经被 ros2 launch 的主 executor 持有，重复 add_node 会抛出：
    //   "Node has already been added to an executor"（std::runtime_error）
    //
    // 正确做法：让服务回调在独立 CallbackGroup + Executor 线程中处理，
    // startup_sequence 线程只需 future.wait_for() 等待即可，无需自己 spin。
    // ─────────────────────────────────────────────────────────────────
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    for (const auto & name : node_names) {
        ManagedNode mn;
        mn.name = name;
        mn.change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/" + name + "/change_state",
            rclcpp::ServicesQoS(),
            service_cb_group_);
        mn.get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(
            "/" + name + "/get_state",
            rclcpp::ServicesQoS(),
            service_cb_group_);
        managed_nodes_.push_back(std::move(mn));
    }

    // 把服务 CallbackGroup 加入专用 executor（不是整个节点）
    service_executor_.add_callback_group(
        service_cb_group_, this->get_node_base_interface());

    // 启动服务 executor 线程（持续 spin，处理服务响应）
    service_executor_thread_ = std::thread([this]() {
        service_executor_.spin();
    });

    // 启动序列在独立线程执行，startup_sequence 内用 future.wait_for() 等待响应
    startup_thread_ = std::thread([this]() {
        // 给被管理节点一些时间初始化
        std::this_thread::sleep_for(1s);
        startup_sequence();
    });

    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] 将管理 %zu 个节点，启动序列将在 1 秒后开始",
        managed_nodes_.size());
}

LifecycleManagerNode::~LifecycleManagerNode()
{
    // 先停止 startup 线程
    if (startup_thread_.joinable()) {
        startup_thread_.join();
    }
    // 再停止 service executor
    service_executor_.cancel();
    if (service_executor_thread_.joinable()) {
        service_executor_thread_.join();
    }
}

// ──────────────────────────────────────────────────────────────
// 主要启动序列：等待服务 → configure all → 延迟 → activate all
// 在独立 std::thread 中运行，future.wait_for() 等待服务响应
// ──────────────────────────────────────────────────────────────
void LifecycleManagerNode::startup_sequence()
{
    // 1. 等待所有节点服务就绪
    for (const auto & mn : managed_nodes_) {
        RCLCPP_INFO(get_logger(),
            "[LifecycleManager] 等待节点 '%s' 的服务...", mn.name.c_str());
        if (!wait_for_service_ready(mn, 5s)) {
            RCLCPP_ERROR(get_logger(),
                "[LifecycleManager] 节点 '%s' 服务超时，放弃启动序列", mn.name.c_str());
            return;
        }
    }

    // 2. Configure all
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── CONFIGURE ALL ──");
    for (const auto & mn : managed_nodes_) {
        const uint8_t current = get_state(mn);
        RCLCPP_INFO(get_logger(),
            "[LifecycleManager] '%s' 当前状态 id=%u，发送 configure...",
            mn.name.c_str(), current);

        if (current != State::PRIMARY_STATE_UNCONFIGURED) {
            RCLCPP_WARN(get_logger(),
                "[LifecycleManager] '%s' 不在 Unconfigured 状态，跳过 configure",
                mn.name.c_str());
            continue;
        }
        if (!change_state(mn, Transition::TRANSITION_CONFIGURE)) {
            RCLCPP_ERROR(get_logger(),
                "[LifecycleManager] '%s' configure 失败，放弃后续步骤", mn.name.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[LifecycleManager] '%s' configure ✓", mn.name.c_str());
    }

    // 3. 等待 transition_delay_sec 后 activate
    const double delay = this->get_parameter("transition_delay_sec").as_double();
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] 等待 %.1f 秒后 activate...", delay);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int64_t>(delay * 1000)));

    // 4. Activate all
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── ACTIVATE ALL ──");
    for (const auto & mn : managed_nodes_) {
        const uint8_t current = get_state(mn);
        RCLCPP_INFO(get_logger(),
            "[LifecycleManager] '%s' 当前状态 id=%u，发送 activate...",
            mn.name.c_str(), current);

        if (current != State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_WARN(get_logger(),
                "[LifecycleManager] '%s' 不在 Inactive 状态，跳过 activate",
                mn.name.c_str());
            continue;
        }
        if (!change_state(mn, Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_ERROR(get_logger(),
                "[LifecycleManager] '%s' activate 失败", mn.name.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[LifecycleManager] '%s' activate ✓", mn.name.c_str());
    }

    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── 所有节点已 Active ── 管理器进入待机状态");
}

// ──────────────────────────────────────────────────────────────
// 辅助：等待服务就绪（在 startup_thread_ 中调用，可以阻塞）
// ──────────────────────────────────────────────────────────────
bool LifecycleManagerNode::wait_for_service_ready(
    const ManagedNode & node, std::chrono::seconds timeout)
{
    if (!node.change_state_client->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] change_state 服务不可用：%s", node.name.c_str());
        return false;
    }
    if (!node.get_state_client->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] get_state 服务不可用：%s", node.name.c_str());
        return false;
    }
    return true;
}

// ──────────────────────────────────────────────────────────────
// 辅助：发送 change_state 请求
// service_executor_thread_ 在后台处理响应，future.wait_for() 等待结果
// ──────────────────────────────────────────────────────────────
bool LifecycleManagerNode::change_state(
    const ManagedNode & node, uint8_t transition_id)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = node.change_state_client->async_send_request(request);

    // service_executor_thread_ 正在 spin service_cb_group_，无需手动 spin
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] change_state(%u) 超时：%s",
            transition_id, node.name.c_str());
        return false;
    }
    return future.get()->success;
}

// ──────────────────────────────────────────────────────────────
// 辅助：查询当前状态 id
// ──────────────────────────────────────────────────────────────
uint8_t LifecycleManagerNode::get_state(const ManagedNode & node)
{
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future  = node.get_state_client->async_send_request(request);

    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] get_state 超时：%s", node.name.c_str());
        return State::PRIMARY_STATE_UNKNOWN;
    }
    return future.get()->current_state.id;
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::LifecycleManagerNode)
