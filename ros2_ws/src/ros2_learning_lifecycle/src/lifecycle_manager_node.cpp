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
    // 声明接收容器，将由 load_managed_node_names 填充受管节点名列表。
    std::vector<std::string> node_names;
    // 声明并加载 managed_nodes / transition_delay_sec，确保管理器有完整启动配置。
    load_managed_node_names(node_names);
    // 提前准备独立的服务回调执行环境，避免与其它回调互相阻塞。
    setup_service_executor();
    // 为每个受管节点创建 change_state/get_state 客户端，建立控制通道。
    initialize_managed_nodes(node_names);
    // 启动后台线程：一个负责服务回调执行，一个负责启动序列调度。
    start_background_threads();

    // 输出最终纳管节点数量，便于确认参数生效并定位启动问题。
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] 将管理 %zu 个节点，启动序列将在 1 秒后开始",
        managed_nodes_.size());
}

/**
 * @brief 停止并回收后台线程资源。
 */
LifecycleManagerNode::~LifecycleManagerNode()
{
    // 先回收启动线程，确保不再有新的状态迁移动作发生。
    if (startup_thread_.joinable()) {
        startup_thread_.join();
    }
    // 再取消服务执行器，通知 spin 循环退出以便安全结束后台服务线程。
    service_executor_.cancel();
    // 等待服务执行线程完全退出，避免析构后仍访问节点资源。
    if (service_executor_thread_.joinable()) {
        service_executor_thread_.join();
    }
}

/**
 * @brief 执行完整启动序列：等待服务、configure 全部、延迟后 activate 全部。
 */
void LifecycleManagerNode::startup_sequence()
{
    // 第一步先确认所有依赖服务可用，避免后续请求在服务未上线时失败。
    if (!wait_for_all_services(5s)) {
        return;
    }
    // 第二步统一执行 configure，将节点从 Unconfigured 推进到 Inactive。
    if (!configure_all_nodes()) {
        return;
    }
    // 在 configure 与 activate 之间按参数等待，给节点内部初始化留出稳定时间。
    wait_transition_delay();
    // 最后执行 activate，将可运行节点切换到 Active。
    if (!activate_all_nodes()) {
        return;
    }

    // 全流程成功后输出状态，表示管理器已完成启动编排并进入待机。
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── 所有节点已 Active ── 管理器进入待机状态");
}

/**
 * @brief 声明参数并读取受管节点列表。
 * @param[out] node_names 读取到的受管节点名称列表。
 */
void LifecycleManagerNode::load_managed_node_names(std::vector<std::string> & node_names)
{
    // 声明受管节点参数并给出默认值，保证未显式配置时也能按示例拓扑运行。
    this->declare_parameter<std::vector<std::string>>(
        "managed_nodes", {"sensor_node", "processor_node"});
    // 声明阶段切换延时参数，用于控制 configure->activate 的节奏。
    this->declare_parameter("transition_delay_sec", 1.0);
    // 读取参数并输出给调用方，后续将据此创建客户端和执行状态迁移。
    node_names = this->get_parameter("managed_nodes").as_string_array();
}

/**
 * @brief 为每个受管节点创建 lifecycle 服务客户端。
 * @param[in] node_names 受管节点名称列表。
 */
void LifecycleManagerNode::initialize_managed_nodes(const std::vector<std::string> & node_names)
{
    // 遍历参数中的每个节点名，逐一构建其生命周期服务访问能力。
    for (const auto & name : node_names) {
        // 创建临时 ManagedNode 容器，收集节点名与两类服务客户端。
        ManagedNode node;
        node.name = name;
        // 绑定 change_state 服务客户端，用于触发 configure/activate 等迁移。
        node.change_state_client = this->create_client<ChangeState>(
            "/" + name + "/change_state",
            rclcpp::ServicesQoS(),
            service_cb_group_);
        // 绑定 get_state 服务客户端，用于读取当前状态并做迁移前置判断。
        node.get_state_client = this->create_client<GetState>(
            "/" + name + "/get_state",
            rclcpp::ServicesQoS(),
            service_cb_group_);
        // 将完整节点描述写入管理列表，供启动序列按顺序处理。
        managed_nodes_.push_back(std::move(node));
    }
}

/**
 * @brief 配置服务回调组并绑定到专用执行器。
 */
void LifecycleManagerNode::setup_service_executor()
{
    // 创建互斥回调组，确保生命周期服务回调按受控方式串行执行。
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // 将回调组挂到专用执行器，避免与默认执行上下文竞争。
    service_executor_.add_callback_group(
        service_cb_group_, this->get_node_base_interface());
}

/**
 * @brief 启动服务执行器线程与启动序列线程。
 */
void LifecycleManagerNode::start_background_threads()
{
    // 启动服务执行线程，持续 spin 以驱动 async_send_request 的响应回调。
    service_executor_thread_ = std::thread([this]() {
        service_executor_.spin();
    });

    // 延迟 1 秒后执行生命周期编排，给其他节点和服务创建留出时间窗口。
    startup_thread_ = std::thread([this]() {
        std::this_thread::sleep_for(1s);
        // 延迟后进入统一启动流程，按 configure->activate 顺序执行。
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
    // 对每个受管节点逐一探测服务可用性，失败即中止整体启动序列。
    for (const auto & node : managed_nodes_) {
        // 先打日志标记当前等待对象，方便定位卡住或超时的节点。
        RCLCPP_INFO(get_logger(),
            "[LifecycleManager] 等待节点 '%s' 的服务...", node.name.c_str());
        // 同时要求 change_state 与 get_state 都可用，保证后续控制与查询都可执行。
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
    // 标记进入批量 configure 阶段，便于从日志上区分生命周期阶段。
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── CONFIGURE ALL ──");
    // 按纳管顺序逐个配置；任一节点失败时立即停止，避免系统处于部分不一致状态。
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
    // 读取用户配置的阶段间隔时间，允许按场景调节系统收敛速度。
    const double delay_sec = this->get_parameter("transition_delay_sec").as_double();
    // 打印实际等待时长，便于确认参数是否被正确加载。
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] 等待 %.1f 秒后 activate...", delay_sec);

    // 将秒转换为毫秒并阻塞等待，确保 activate 不会过早触发。
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int64_t>(delay_sec * 1000.0)));
}

/**
 * @brief 顺序激活所有处于 Inactive 的节点。
 * @return 全部需要激活的节点均成功返回 true，否则返回 false。
 */
bool LifecycleManagerNode::activate_all_nodes()
{
    // 标记进入批量 activate 阶段，帮助排查 configure 后的激活问题。
    RCLCPP_INFO(get_logger(), "[LifecycleManager] ── ACTIVATE ALL ──");
    // 顺序激活所有节点；若某节点失败则立即终止，避免继续扩大故障面。
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
    // 先读取节点当前生命周期状态，避免盲目发送不匹配的迁移请求。
    const uint8_t current_state = get_state(node);
    // 记录状态与即将执行动作，便于在失败时追溯上下文。
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] '%s' 当前状态 id=%u，发送 configure...",
        node.name.c_str(), current_state);

    // 只有 Unconfigured 才允许 configure；其余状态直接跳过以保持幂等。
    if (current_state != State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_WARN(get_logger(),
            "[LifecycleManager] '%s' 不在 Unconfigured 状态，跳过 configure",
            node.name.c_str());
        return true;
    }

    // 发起 configure 状态迁移；若失败则中止整体流程，防止继续向下执行。
    if (!change_state(node, Transition::TRANSITION_CONFIGURE)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] '%s' configure 失败，放弃后续步骤", node.name.c_str());
        return false;
    }

    // 记录单节点 configure 成功，作为后续 activate 的前置确认。
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
    // 查询当前状态，确保 activate 仅在合法前置状态下执行。
    const uint8_t current_state = get_state(node);
    // 记录当前状态和目标动作，便于分析生命周期切换链路。
    RCLCPP_INFO(get_logger(),
        "[LifecycleManager] '%s' 当前状态 id=%u，发送 activate...",
        node.name.c_str(), current_state);

    // 只有 Inactive 才进入 activate；否则保持现状并跳过，避免非法迁移。
    if (current_state != State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_WARN(get_logger(),
            "[LifecycleManager] '%s' 不在 Inactive 状态，跳过 activate",
            node.name.c_str());
        return true;
    }

    // 请求执行 activate 迁移；失败时返回 false 让上层及时停止批处理。
    if (!change_state(node, Transition::TRANSITION_ACTIVATE)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] '%s' activate 失败", node.name.c_str());
        return false;
    }

    // 记录激活完成，表示该节点已达到可运行状态。
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
    // 先检查 change_state 服务是否在线，因为状态迁移是管理器的核心控制入口。
    if (!node.change_state_client->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] change_state 服务不可用：%s", node.name.c_str());
        return false;
    }
    // 再检查 get_state 服务，确保后续状态查询与迁移前判断可正常执行。
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
    // 构造 change_state 请求并填入目标 transition id。
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;

    // 发送异步请求，响应由专用 service executor 线程驱动处理。
    auto future = node.change_state_client->async_send_request(request);

    // service_executor_thread_ 正在 spin service_cb_group_，无需手动 spin
    // 对服务响应设置超时保护，避免单节点阻塞导致整个启动流程卡死。
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] change_state(%u) 超时：%s",
            transition_id, node.name.c_str());
        return false;
    }
    // 返回服务端 success 字段作为状态迁移最终结果。
    return future.get()->success;
}

/**
 * @brief 查询被管理节点当前生命周期主状态 ID。
 * @param[in] node 被管理节点。
 * @return 当前状态 ID；超时则返回 PRIMARY_STATE_UNKNOWN。
 */
uint8_t LifecycleManagerNode::get_state(const ManagedNode & node)
{
    // 构造空的 get_state 请求，用于查询节点当前生命周期主状态。
    auto request = std::make_shared<GetState::Request>();
    // 异步发送请求，避免在调用处显式执行器 spin。
    auto future  = node.get_state_client->async_send_request(request);

    // 设置查询超时，防止状态读取异常拖慢整个生命周期编排。
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(),
            "[LifecycleManager] get_state 超时：%s", node.name.c_str());
        // 超时返回 UNKNOWN，让上层逻辑以保守策略处理（通常会跳过迁移）。
        return State::PRIMARY_STATE_UNKNOWN;
    }
    // 返回节点当前主状态 id，供 configure/activate 阶段做状态判断。
    return future.get()->current_state.id;
}

}  // namespace ros2_learning_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_lifecycle::LifecycleManagerNode)
