#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// 引入我们的自定义节点头文件
#include "ros2_learning_behavior_tree/nodes/move_base_node.hpp"
#include "ros2_learning_behavior_tree/nodes/simple_arm_action.hpp"
#include "ros2_learning_behavior_tree/nodes/mock_move_base.hpp"
#include "ros2_learning_behavior_tree/nodes/mock_recovery.hpp"
#include "ros2_learning_behavior_tree/nodes/get_location_from_queue.hpp"

// 使用 ament_index_cpp 来查找 share 目录（定位行为树 XML 文件）
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace ros2_learning_behavior_tree;

int main(int argc, char ** argv)
{
  // 1. 初始化 ROS 2 环境
  rclcpp::init(argc, argv);

  // 创建一个标准的 ROS 2 节点，用于处理参数、日志输出和后端同步
  auto node = rclcpp::Node::make_shared("bt_demo_node");

  // 2. 创建 BehaviorTreeFactory (行为树工厂)
  // 工厂负责管理所有可用的节点类型，并根据 XML ID 在运行时实例化它们
  BT::BehaviorTreeFactory factory;

  // 3. 注册我们的自定义节点
  
  // --- A. 注入式注册 (registerBuilder) ---
  // 对于需要传入外部对象（如 ROS 节点指针）的节点，我们使用 Lambda 表达式进行构造
  
  // 注册真实的导航节点
  factory.registerBuilder<MoveBase>("MoveBase", 
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MoveBase>(name, config, node);
    });

  // 注册真实的机械臂动作节点
  factory.registerBuilder<SimpleArmAction>("SimpleArmAction",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SimpleArmAction>(name, config, node);
    });

  // --- B. 注册 Mock 模拟节点 (用于脱离仿真环境进行逻辑验证) ---
  factory.registerBuilder<MockMoveBase>("MockMoveBase",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MockMoveBase>(name, config, node);
    });

  factory.registerBuilder<MockRecovery>("MockRecovery",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MockRecovery>(name, config, node);
    });

  // 队列取点节点也需要 node 指针来打印带时间戳的 ROS 日志
  factory.registerBuilder<GetLocationFromQueue>("GetLocationFromQueue",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GetLocationFromQueue>(name, config, node);
    });

  // 4. 配置与加载 XML 文件
  
  // 知识点：通过 ROS 参数动态指定要运行的行为树文件名 (默认 main_tree_composition.xml)
  // 这样无需重新编译即可切换不同的业务逻辑
  node->declare_parameter("tree_file", "main_tree_composition.xml");
  std::string tree_file = node->get_parameter("tree_file").as_string();

  std::string pkg_share = ament_index_cpp::get_package_share_directory("ros2_learning_behavior_tree");
  
  // A. 注册子树定义 (SubTree)
  // 子树是可重用的逻辑单元。必须先注册子树文件，主树才能成功解析 <SubTree ID="..."> 标签。
  std::string subtree_path = pkg_share + "/behavior_trees/fetch_subtree.xml";
  factory.registerBehaviorTreeFromFile(subtree_path);
  RCLCPP_INFO(node->get_logger(), "[System] 子树库已挂载: %s", subtree_path.c_str());

  // B. 注册主树
  std::string main_tree_path = pkg_share + "/behavior_trees/" + tree_file;
  RCLCPP_INFO(node->get_logger(), "[System] 正在加载主业务树: %s", main_tree_path.c_str());
  factory.registerBehaviorTreeFromFile(main_tree_path);
  
  // 5. 实例化行为树 (Instantiation)
  // 根据主树文件中的 ID 创建节点对象树。此过程会进行端口连接和语法检查。
  auto tree = factory.createTree("MainTree");

  // 6. (可选) 可视化调试：Groot2 数据推送
  // 启用后，在 Groot2 软件中输入 localhost (端口 1667) 即可看到实时运行的变色边框和数据流。
  BT::Groot2Publisher publisher(tree);

  // 7. 核心 Tick 循环
  RCLCPP_INFO(node->get_logger(), "[System] 行为树已进入就绪状态，开始 Tick 循环。");

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  // 设置执行频率 (10Hz)，兼顾实时性与系统开销
  rclcpp::Rate rate(10);

  // 循环运行，直到行为树返回 SUCCESS 或 FAILURE 终止状态
  while (rclcpp::ok() && status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::FAILURE) {
    // Tick 一次整棵树
    status = tree.tickOnce();

    // 重要：必须调用 spin_some 处理 ROS 的异步回调！
    // 否则 Action Client 的结果通知永远无法被处理，导致异步节点卡死在 RUNNING 状态。
    rclcpp::spin_some(node);

    rate.sleep();
  }

  // 8. 任务结束处理
  RCLCPP_INFO(node->get_logger(), "[System] 任务执行完毕，最终汇报状态: %s", toStr(status).c_str());

  rclcpp::shutdown();
  return 0;
}
