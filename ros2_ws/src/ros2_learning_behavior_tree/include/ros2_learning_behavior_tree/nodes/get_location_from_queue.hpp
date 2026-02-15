#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>

namespace ros2_learning_behavior_tree
{

/**
 * @brief 队列取点节点 (GetLocationFromQueue)
 * 
 * 这是一个 SyncActionNode (同步动作节点)。
 * 与异步节点不同，同步节点在一次 Tick 中必须立即返回 SUCCESS 或 FAILURE，不允许返回 RUNNING。
 * 它通常用于简单的逻辑计算或瞬间完成的数据处理。
 * 
 * 在本项目中，它充当“生产者”，从预定义队列取出一个地点名称并写入黑板（Blackboard）。
 */
class GetLocationFromQueue : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name 节点在 XML 中的实例名称
   * @param config 节点的配置信息
   * @param node_ptr ROS 节点指针，用于打印日志
   */
  GetLocationFromQueue(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  /**
   * @brief 定义节点提供的端口列表
   * 包含一个 OutputPort，名为 "target_location"。
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 核心执行函数
   * 对于同步节点，没有 onStart/onRunning，只有 tick()。
   * @return 执行状态，必须是 SUCCESS 或 FAILURE。
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;            // ROS 节点指针
  std::vector<std::string> location_queue_; // 模拟的目标地点队列
  size_t current_index_ = 0;               // 当前取到的队列索引
};

} // namespace ros2_learning_behavior_tree
