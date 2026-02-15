#include "ros2_learning_behavior_tree/nodes/get_location_from_queue.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_behavior_tree
{

GetLocationFromQueue::GetLocationFromQueue(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::SyncActionNode(name, config), node_(node_ptr)
{
  // 初始化地点队列，模拟一个待执行的任务列表
  location_queue_ = {"Kitchen (厨房)", "Bedroom (卧室)", "Balcony (阳台)", "Dock (充电桩)"};
}

BT::PortsList GetLocationFromQueue::providedPorts()
{
  // 定义该节点的产出物：一个字符串类型的 target_location
  return {
    BT::OutputPort<std::string>("target_location", "即将前往的目标点名称")
  };
}

BT::NodeStatus GetLocationFromQueue::tick()
{
  // 数据保护：如果队列为空则报错
  if (location_queue_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[GetLocation] 队列为空，无法取点！");
    return BT::NodeStatus::FAILURE;
  }

  // 1. 从队列取出当前指向的目标
  std::string target = location_queue_[current_index_];
  
  // 2. 更新索引，实现循环队列效果
  current_index_ = (current_index_ + 1) % location_queue_.size();

  // 3. 核心机制：将数据写入输出端口 (OutputPort)
  // 在 XML 中，如果配置了 target_location="{next_goal}"，
  // 则这个 target 字符串会被存入黑板变量 next_goal 中。
  setOutput("target_location", target);

  // 4. 打印执行日志
  RCLCPP_INFO(node_->get_logger(), "[GetLocation] 🎲 从队列中取出目标: %s", target.c_str());

  // 5. 同步节点成功返回 SUCCESS
  return BT::NodeStatus::SUCCESS;
}

} // namespace ros2_learning_behavior_tree
