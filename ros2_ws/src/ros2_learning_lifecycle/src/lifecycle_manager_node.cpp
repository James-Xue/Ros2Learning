#include "ros2_learning_lifecycle/lifecycle_manager_node.hpp"

/**
 * @file lifecycle_manager_node.cpp
 * @brief LifecycleManagerNode 生命周期编排实现。
 */

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

/**
 * @brief 构造管理器节点并创建服务客户端、执行器线程与启动线程。
 * @param[in] options ROS 2 节点选项。
 */
LifecycleManagerNode::LifecycleManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lifecycle_manager_node", options)
{
    std::vector<std::string> node_names;
    load_managed_node_names(node_names);
    setup_service_executor();
    initialize_managed_nodes(node_names);
    start_background_threads();

    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] 将管理 %zu 个节点，启动序列将在 1 秒后开始",
        managed_nodes_.size());
}

/**
 * @brief 停止并回收后台线程资源。
 */
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

/**
 * @brief 执行完整启动序列：等待服务、configure 全部、延迟后 activate 全部。
 */
void LifecycleManagerNode::startup_sequence()
{
    if (!wait_for_all_services(5s)) {
        return;
    }
    if (!configure_all_nodes()) {
        return;
    }
    wait_transition_delay();
    if (!activate_all_nodes()) {
        return;
    }

    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── 所有节点已 Active ── 管理器进入待机状态");
}

/**
 * @brief 声明参数并读取受管节点列表。
 * @param[out] node_names 读取到的受管节点名称列表。
 */
void LifecycleManagerNode::load_managed_node_names(std::vector<std::string> & node_names)
{
    this->declare_parameter<std::vector<std::string>>(
        "managed_nodes", {"sensor_node", "processor_node"});
    this->declare_parameter("transition_delay_sec", 1.0);
    node_names = this->get_parameter("managed_nodes").as_string_array();
}

/**
 * @brief 为每个受管节点创建 lifecycle 服务客户端。
 * @param[in] node_names 受管节点名称列表。
 */
void LifecycleManagerNode::initialize_managed_nodes(const std::vector<std::string> & node_names)
{
    for (const auto & name : node_names) {
        ManagedNode node;
        node.name = name;
        node.change_state_client = this->create_client<ChangeState>(
            "/" + name + "/change_state",
            rclcpp::ServicesQoS(),
            service_cb_group_);
        node.get_state_client = this->create_client<GetState>(
            "/" + name + "/get_state",
            rclcpp::ServicesQoS(),
            service_cb_group_);
        managed_nodes_.push_back(std::move(node));
    }
}

/**
 * @brief 配置服务回调组并绑定到专用执行器。
 */
void LifecycleManagerNode::setup_service_executor()
{
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    service_executor_.add_callback_group(
        service_cb_group_, this->get_node_base_interface());
}

/**
 * @brief 启动服务执行器线程与启动序列线程。
 */
void LifecycleManagerNode::start_background_threads()
{
    service_executor_thread_ = std::thread([this]() {
        service_executor_.spin();
    });

    startup_thread_ = std::thread([this]() {
        std::this_thread::sleep_for(1s);
        startup_sequence();
    });
}

/**
 * @brief 等待所有被管理节点生命周期服务就绪。
 * @param[in] timeout 每个节点服务等待超时时间。
 * @return 全部节点服务就绪返回 true，否则返回 false。
 */
bool LifecycleManagerNode::wait_for_all_services(std::chrono::seconds timeout)
{
    for (const auto & node : managed_nodes_) {
        RCLCPP_INFO(get_logger(),
            "[LifecycleManager] 等待节点 '%s' 的服务...", node.name.c_str());
        if (!wait_for_service_ready(node, timeout)) {
            RCLCPP_ERROR(get_logger(),
                "[LifecycleManager] 节点 '%s' 服务超时，放弃启动序列", node.name.c_str());
            return false;
        }
    }
    return true;
}

/**
 * @brief 顺序配置所有处于 Unconfigured 的节点。
 * @return 全部需要配置的节点均成功返回 true，否则返回 false。
 */
bool LifecycleManagerNode::configure_all_nodes()
{
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── CONFIGURE ALL ──");
    for (const auto & node : managed_nodes_) {
        if (!configure_node_if_needed(node)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 按参数延迟启动下一阶段状态迁移。
 */
void LifecycleManagerNode::wait_transition_delay()
{
    const double delay_sec = this->get_parameter("transition_delay_sec").as_double();
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] 等待 %.1f 秒后 activate...", delay_sec);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int64_t>(delay_sec * 1000.0)));
}

/**
 * @brief 顺序激活所有处于 Inactive 的节点。
 * @return 全部需要激活的节点均成功返回 true，否则返回 false。
 */
bool LifecycleManagerNode::activate_all_nodes()
{
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── ACTIVATE ALL ──");
    for (const auto & node : managed_nodes_) {
        if (!activate_node_if_needed(node)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 执行单节点的 configure 迁移（仅在状态匹配时执行）。
 * @param[in] node 被管理节点描述。
 * @return 迁移成功或无需迁移返回 true，迁移失败返回 false。
 */
bool LifecycleManagerNode::configure_node_if_needed(const ManagedNode & node)
{
    const uint8_t current_state = get_state(node);
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] '%s' 当前状态 id=%u，发送 configure...",
        node.name.c_str(), current_state);

    if (current_state != State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_WARN(get_logger(),
            "[LifecycleManager] '%s' 不在 Unconfigured 状态，跳过 configure",
            node.name.c_str());
        return true;
    }

    if (!change_state(node, Transition::TRANSITION_CONFIGURE)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] '%s' configure 失败，放弃后续步骤", node.name.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "[LifecycleManager] '%s' configure ✓", node.name.c_str());
    return true;
}

/**
 * @brief 执行单节点的 activate 迁移（仅在状态匹配时执行）。
 * @param[in] node 被管理节点描述。
 * @return 迁移成功或无需迁移返回 true，迁移失败返回 false。
 */
bool LifecycleManagerNode::activate_node_if_needed(const ManagedNode & node)
{
    const uint8_t current_state = get_state(node);
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] '%s' 当前状态 id=%u，发送 activate...",
        node.name.c_str(), current_state);

    if (current_state != State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_WARN(get_logger(),
            "[LifecycleManager] '%s' 不在 Inactive 状态，跳过 activate",
            node.name.c_str());
        return true;
    }

    if (!change_state(node, Transition::TRANSITION_ACTIVATE)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] '%s' activate 失败", node.name.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "[LifecycleManager] '%s' activate ✓", node.name.c_str());
    return true;
}

/**
 * @brief 等待单个被管理节点的 get_state/change_state 服务可用。
 * @param[in] node 被管理节点。
 * @param[in] timeout 服务等待超时时间。
 * @return 服务均可用返回 true，否则返回 false。
 */
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

/**
 * @brief 发送 change_state 服务请求。
 * @param[in] node 被管理节点。
 * @param[in] transition_id 生命周期转换 ID。
 * @return 转换成功返回 true，否则返回 false。
 */
bool LifecycleManagerNode::change_state(
    const ManagedNode & node, uint8_t transition_id)
{
    auto request = std::make_shared<ChangeState::Request>();
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

/**
 * @brief 查询被管理节点当前生命周期主状态 ID。
 * @param[in] node 被管理节点。
 * @return 当前状态 ID；超时则返回 PRIMARY_STATE_UNKNOWN。
 */
uint8_t LifecycleManagerNode::get_state(const ManagedNode & node)
{
    auto request = std::make_shared<GetState::Request>();
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
