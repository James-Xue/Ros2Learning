#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// 引入我们的自定义节点
#include "ros2_learning_behavior_tree/nodes/move_base_node.hpp"
#include "ros2_learning_behavior_tree/nodes/simple_arm_action.hpp"

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
  // 知识点：
  // - registerBuilder: 用于注册需要复杂构造函数（如需要传入 ros_node）的节点。
  // - registerNodeType: 用于注册默认构造函数的简单节点。

  // 注册 MoveBase (Nav2 客户端)
  factory.registerBuilder<MoveBase>(
    "MoveBase", // XML 中的标签名 <MoveBase ...>
    [node](const std::string & name, const BT::NodeConfig & config) {
      // 在这里我们将 ROS 节点指针注入到 BT 节点中
      return std::make_unique<MoveBase>(name, config, node);
    });

  // 注册 SimpleArmAction (机械臂模拟)
  factory.registerBuilder<SimpleArmAction>(
    "SimpleArmAction",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SimpleArmAction>(name, config, node);
    });

  // 4. 确定 XML 文件路径
  // 我们使用 ament_index_cpp 找到安装后的 share 目录，确保在任何目录下运行都能找到文件
  std::string pkg_share =
    ament_index_cpp::get_package_share_directory("ros2_learning_behavior_tree");
  std::string xml_path = pkg_share + "/behavior_trees/simple_patrol.xml";

  RCLCPP_INFO(node->get_logger(), "正在加载行为树文件: %s", xml_path.c_str());

  // 5. 创建树 (Instantiation)
  // 工厂会解析 XML，并根据注册的构建器创建所有节点对象
  auto tree = factory.createTreeFromFile(xml_path);

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
