#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ros2_learning_behavior_tree/nodes/move_base_node.hpp"
#include "ros2_learning_behavior_tree/nodes/simple_arm_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace ros2_learning_behavior_tree;

class BehaviorTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_bt_node");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(BehaviorTreeTest, TestTreeInstantiation)
{
  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<MoveBase>(
    "MoveBase",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MoveBase>(name, config, node_);
    });

  factory.registerBuilder<SimpleArmAction>(
    "SimpleArmAction",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SimpleArmAction>(name, config, node_);
    });

  std::string pkg_share =
    ament_index_cpp::get_package_share_directory("ros2_learning_behavior_tree");
  std::string xml_path = pkg_share + "/behavior_trees/simple_patrol.xml";

  // 不抛出异常即为通过
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(xml_path);
  });
}
