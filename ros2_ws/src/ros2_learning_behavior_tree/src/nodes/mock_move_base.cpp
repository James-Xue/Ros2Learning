#include "ros2_learning_behavior_tree/nodes/mock_move_base.hpp"
#include <random>

namespace ros2_learning_behavior_tree
{

MockMoveBase::MockMoveBase(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
  // 初始化 C++11 标准随机数生成器（梅森旋转算法）
  std::random_device rd;
  gen_ = std::mt19937(rd());
  dis_ = std::uniform_real_distribution<>(0.0, 1.0);
}

BT::PortsList MockMoveBase::providedPorts()
{
  // 定义该节点在行为树中的“插槽” (Ports)
  return {
    BT::InputPort<std::string>("location", "Target A", "目标地点名称"),
    BT::InputPort<double>("probability", 1.0, "成功概率 (0.0 - 1.0)"),
    BT::InputPort<int>("duration", 2000, "模拟任务耗时 (ms)")
  };
}

BT::NodeStatus MockMoveBase::onStart()
{
  // 知识点：从端口获取数据 (getInput)
  // 如果输入失败（通常是因为 XML 没配置或者格式不对），这里会抛出异常或返回空
  if (!getInput<std::string>("location", location_)) {
    throw BT::RuntimeError("缺少关键参数: location");
  }
  if (!getInput<double>("probability", probability_)) {
    probability_ = 1.0; // 如果没给概率，默认 100% 成功
  }
  if (!getInput<int>("duration", duration_ms_)) {
    duration_ms_ = 2000; // 如果没给耗时，默认 2 秒
  }

  // 记录开始运行的时间戳
  start_time_ = std::chrono::system_clock::now();

  RCLCPP_INFO(node_->get_logger(), 
    "[MockMove] 🚀 正在前往 %s (模拟耗时: %dms, 预设成功率: %.1f)...", 
    location_.c_str(), duration_ms_, probability_);

  // 返回 RUNNING 后，行为树调度器会等待下一轮 Tick 继续调用 onRunning
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MockMoveBase::onRunning()
{
  // 计算自启动以来经过的毫秒数
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();

  // 若尚未到达预设耗时，继续保持 RUNNING 状态
  if (elapsed < duration_ms_) {
    return BT::NodeStatus::RUNNING;
  }

  // 模拟任务结束：掷骰子决定成败
  double random_val = dis_(gen_);

  if (random_val <= probability_) {
    RCLCPP_INFO(node_->get_logger(), "[MockMove] ✅ 成功到达 %s！", location_.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "[MockMove] ❌ 导航至 %s 任务失败！(随机值: %.2f > 阈值: %.2f)", 
      location_.c_str(), random_val, probability_);
    // 返回 FAILURE 后，父节点如果是 Sequence，则整个流程会中止，并可能触发 Fallback 分支
    return BT::NodeStatus::FAILURE;
  }
}

void MockMoveBase::onHalted()
{
  // 当该动作被外部事件（如按下急停、或者并行逻辑中另一个节点抢占）强行中止时调用
  RCLCPP_INFO(node_->get_logger(), "[MockMove] 🛑 当前导航任务被中断取消！");
}

} // namespace ros2_learning_behavior_tree
