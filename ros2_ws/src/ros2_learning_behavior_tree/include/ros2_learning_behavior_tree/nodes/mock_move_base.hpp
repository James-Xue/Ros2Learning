#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <random>

namespace ros2_learning_behavior_tree
{

/**
 * @brief 模拟移动节点 (MockMoveBase)
 * 
 * 这是一个 StatefulActionNode (异步动作节点)，模拟耗时的导航过程。
 * 它不仅演示了异步执行，还展示了如何通过 InputPort 动态配置节点的行为（如成功率和耗时）。
 */
class MockMoveBase : public BT::StatefulActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name 节点实例名
   * @param config 节点配置
   * @param node_ptr ROS 节点指针
   */
  MockMoveBase(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  /**
   * @brief 定义端口列表
   * 包括：location (目标)、probability (成功概率)、duration (模拟耗时)
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 启动逻辑：读取端口参数并重置计时器
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief 运行逻辑：检查时间是否到达，并根据成功率返回结果
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief 中断逻辑：处理任务被取消的情况
   */
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;                      // ROS 节点指针
  std::chrono::system_clock::time_point start_time_;  // 开始执行的时间
  int duration_ms_;                                   // 本次导航任务的模拟耗时
  double probability_;                                // 本次任务成功的概率
  std::string location_;                              // 本次任务的目标地点

  // -------------------------------------------------------------------------
  // 知识点：线程安全与随机数
  // 将随机数生成器作为成员变量，避免使用 static 全局变量，从而在多线程执行器下更安全。
  // -------------------------------------------------------------------------
  std::mt19937 gen_;                      // 梅森旋转随机数引擎
  std::uniform_real_distribution<> dis_;  // 0.0 到 1.0 的均匀分布
};

} // namespace ros2_learning_behavior_tree
