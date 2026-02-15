#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_behavior_tree
{

/**
 * @brief 模拟恢复行为节点 (MockRecovery)
 * 
 * 这是一个 StatefulActionNode (状态动作节点)，用于模拟异步的耗时操作。
 * 在行为树中，恢复行为通常用于尝试修复前一个节点发生的故障（例如：原地旋转以清除雷达盲区或清除代价地图）。
 */
class MockRecovery : public BT::StatefulActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name 节点在 XML 中的实例名称
   * @param config 节点的配置信息（包含端口映射等）
   * @param node_ptr ROS 节点指针，用于打印日志
   */
  MockRecovery(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  /**
   * @brief 定义节点提供的端口列表
   * @return 包含输入/输出端口定义的列表
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 节点启动时的钩子函数
   * 当节点从 IDLE 状态转变为 RUNNING 状态时调用一次。
   * 通常用于初始化逻辑、读取输入参数、记录开始时间等。
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief 节点运行时的钩子函数
   * 只要节点处于 RUNNING 状态，每一轮 Tick 都会调用此函数。
   * 用于检查任务是否完成，并返回 RUNNING, SUCCESS 或 FAILURE。
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief 节点被中断时的钩子函数
   * 如果父节点（如 Parallel 或 ReactiveSequence）决定停止此节点，则调用此函数。
   * 用于清理资源、停止底层动作等。
   */
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;                     // ROS 节点指针，用于日志记录
  std::chrono::system_clock::time_point start_time_; // 记录动作开始的时间点
  int duration_ms_ = 1000;                           // 此恢复动作模拟的耗时（毫秒）
};

} // namespace ros2_learning_behavior_tree
