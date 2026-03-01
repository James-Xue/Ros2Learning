/**
 * @file test_lifecycle_transitions.cpp
 * @brief 生命周期状态切换单元测试。
 */

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "ros2_learning_lifecycle/sensor_node.hpp"
#include "ros2_learning_lifecycle/processor_node.hpp"

using LS = lifecycle_msgs::msg::State;
using LT = lifecycle_msgs::msg::Transition;

/**
 * @class LifecycleTransitionTest
 * @brief 生命周期节点状态切换测试夹具。
 */
class LifecycleTransitionTest : public ::testing::Test
{
protected:
    /// @brief 初始化 rclcpp 运行时。
    void SetUp() override { rclcpp::init(0, nullptr); }
    /// @brief 关闭 rclcpp 运行时。
    void TearDown() override { rclcpp::shutdown(); }

    /**
     * @brief 触发指定生命周期转换并返回新状态 ID。
     * @param node 目标生命周期节点。
     * @param transition_id 生命周期转换 ID。
     * @return 转换后的主状态 ID。
     */
    uint8_t do_transition(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node, uint8_t transition_id)
    {
        return node->trigger_transition(
            rclcpp_lifecycle::Transition(transition_id)).id();
    }
};

/**
 * @brief 验证 SensorNode configure 后进入 Inactive。
 */
TEST_F(LifecycleTransitionTest, SensorNodeConfigureSuccess)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    EXPECT_EQ(node->get_current_state().id(), LS::PRIMARY_STATE_UNCONFIGURED);

    const uint8_t new_state = do_transition(node, LT::TRANSITION_CONFIGURE);
    EXPECT_EQ(new_state, LS::PRIMARY_STATE_INACTIVE);
}

/**
 * @brief 验证 SensorNode 完整正向生命周期循环。
 */
TEST_F(LifecycleTransitionTest, SensorNodeFullCycle)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();

    EXPECT_EQ(do_transition(node, LT::TRANSITION_CONFIGURE), LS::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(do_transition(node, LT::TRANSITION_ACTIVATE),  LS::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(do_transition(node, LT::TRANSITION_DEACTIVATE), LS::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(do_transition(node, LT::TRANSITION_CLEANUP),    LS::PRIMARY_STATE_UNCONFIGURED);
}

/**
 * @brief 验证 SensorNode 从 Active 执行 shutdown 后进入 Finalized。
 */
TEST_F(LifecycleTransitionTest, SensorNodeShutdownFromActive)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();

    do_transition(node, LT::TRANSITION_CONFIGURE);
    do_transition(node, LT::TRANSITION_ACTIVATE);
    EXPECT_EQ(node->get_current_state().id(), LS::PRIMARY_STATE_ACTIVE);

    const uint8_t final_state = do_transition(
        node, LT::TRANSITION_ACTIVE_SHUTDOWN);
    EXPECT_EQ(final_state, LS::PRIMARY_STATE_FINALIZED);
}

/**
 * @brief 验证 SensorNode 发布器在 configure 后存在但未激活。
 */
TEST_F(LifecycleTransitionTest, SensorPublisherInactiveAfterConfigure)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    do_transition(node, LT::TRANSITION_CONFIGURE);

    ASSERT_NE(node->get_publisher(), nullptr);
    EXPECT_FALSE(node->get_publisher()->is_activated());
}

/**
 * @brief 验证 SensorNode 发布器在 activate 后已激活。
 */
TEST_F(LifecycleTransitionTest, SensorPublisherActiveAfterActivate)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    do_transition(node, LT::TRANSITION_CONFIGURE);
    do_transition(node, LT::TRANSITION_ACTIVATE);

    ASSERT_NE(node->get_publisher(), nullptr);
    EXPECT_TRUE(node->get_publisher()->is_activated());
}

/**
 * @brief 验证 SensorNode 在 deactivate 后发布器停用。
 */
TEST_F(LifecycleTransitionTest, SensorPublisherInactiveAfterDeactivate)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    do_transition(node, LT::TRANSITION_CONFIGURE);
    do_transition(node, LT::TRANSITION_ACTIVATE);
    do_transition(node, LT::TRANSITION_DEACTIVATE);

    ASSERT_NE(node->get_publisher(), nullptr);
    EXPECT_FALSE(node->get_publisher()->is_activated());
}

/**
 * @brief 验证 ProcessorNode 注入 failure 时 configure 回到 Unconfigured。
 */
TEST_F(LifecycleTransitionTest, ProcessorNodeFailureOnConfigure)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("configure_behavior", "failure");
    auto node = std::make_shared<ros2_learning_lifecycle::ProcessorNode>(opts);

    // configure 失败：节点应回到 Unconfigured（非 Inactive）
    const uint8_t state_after = do_transition(node, LT::TRANSITION_CONFIGURE);
    EXPECT_EQ(state_after, LS::PRIMARY_STATE_UNCONFIGURED);
}

/**
 * @brief 验证 ProcessorNode 注入 error 时通过 on_error 恢复到 Unconfigured。
 */
TEST_F(LifecycleTransitionTest, ProcessorNodeErrorOnConfigure)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("configure_behavior", "error");
    auto node = std::make_shared<ros2_learning_lifecycle::ProcessorNode>(opts);

    // configure 返回 ERROR → 节点进入 ErrorProcessing → on_error 返回 SUCCESS → Unconfigured
    const uint8_t state_after = do_transition(node, LT::TRANSITION_CONFIGURE);
    EXPECT_EQ(state_after, LS::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
