#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// 引入我们的自定义节点
#include "ros2_learning_behavior_tree/nodes/move_base_node.hpp"
#include "ros2_learning_behavior_tree/nodes/simple_arm_action.hpp"
#include "ros2_learning_behavior_tree/nodes/mock_move_base.hpp"
#include "ros2_learning_behavior_tree/nodes/mock_recovery.hpp"
#include "ros2_learning_behavior_tree/nodes/get_location_from_queue.hpp"

// 使用 ament_index_cpp 来查找 share 目录（如果不硬编码路径的话）
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace ros2_learning_behavior_tree;

int main(int argc, char ** argv)
{
  // 1. 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 创建一个标准的 ROS 2 节点，用于处理参数、日志和时间
  auto node = rclcpp::Node::make_shared("bt_demo_node");

  // 2. 创建 BehaviorTreeFactory
  // 工厂是 BT 的核心，负责注册节点类型和实例化树
  BT::BehaviorTreeFactory factory;

  // 3. 注册我们的自定义节点
  
  // --- 真实节点 ---
  factory.registerBuilder<MoveBase>("MoveBase", 
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MoveBase>(name, config, node);
    });

  factory.registerBuilder<SimpleArmAction>("SimpleArmAction",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SimpleArmAction>(name, config, node);
    });

  // --- Mock 节点 ---
  factory.registerBuilder<MockMoveBase>("MockMoveBase",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MockMoveBase>(name, config, node);
    });

  factory.registerBuilder<MockRecovery>("MockRecovery",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MockRecovery>(name, config, node);
    });

  // 现在 GetLocationFromQueue 也需要节点指针了，所以要用 registerBuilder
  factory.registerBuilder<GetLocationFromQueue>("GetLocationFromQueue",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GetLocationFromQueue>(name, config, node);
    });

  // 4. 加载行为树文件
  // 声明参数 tree_file，允许通过 launch 文件修改要运行的树
  node->declare_parameter("tree_file", "main_tree_composition.xml");
  std::string tree_file = node->get_parameter("tree_file").as_string();

  std::string pkg_share = ament_index_cpp::get_package_share_directory("ros2_learning_behavior_tree");
  
  // 注册子树 (固定依赖)
  std::string subtree_path = pkg_share + "/behavior_trees/fetch_subtree.xml";
  factory.registerBehaviorTreeFromFile(subtree_path);
  RCLCPP_INFO(node->get_logger(), "已注册子树: %s", subtree_path.c_str());

  // 注册主树 (动态选择)
  std::string main_tree_path = pkg_share + "/behavior_trees/" + tree_file;
  RCLCPP_INFO(node->get_logger(), "正在加载主树: %s", main_tree_path.c_str());
  
  // 这里我们假设所有的主树文件都包含一个 ID 为 "MainTree" 的树
  // 如果你想更灵活，可以解析 XML 获取 ID，或者参数化 TreeID
  factory.registerBehaviorTreeFromFile(main_tree_path);
  
  // 5. 实例化主树
  auto tree = factory.createTree("MainTree");

  // 6. (可选) 添加 Groot2 发布器，用于可视化调试
  // 默认端口 1667。启用后，打开 Groot2 软件 -> Connect 即可看到实时运行状态。
  // const unsigned port = 1667;
  // BT::Groot2Publisher publisher(tree, port);

  // 7. 执行树 (Execution Loop)
  RCLCPP_INFO(node->get_logger(), "行为树开始运行...");

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  // 设置 tick 频率 (例如 10Hz)
  // 频率不用太高，因为大部分时间是等待 Action Server 反馈
  rclcpp::Rate rate(10);

  while (rclcpp::ok() && status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE) {
    // 核心：Tick 一次树的根节点
    // 树会递归地 tick 子节点，直到返回 RUNNING, SUCCESS 或 FAILURE
    status = tree.tickOnce();

    // 重要：必须调用 spin_some 来处理 ROS 回调！
    // 否则 Action Client 的 result_callback 永远不会被触发，MoveBase 会一直卡在 RUNNING
    rclcpp::spin_some(node);

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "行为树运行结束，最终状态: %s", toStr(status).c_str());

  rclcpp::shutdown();
  return 0;
}
