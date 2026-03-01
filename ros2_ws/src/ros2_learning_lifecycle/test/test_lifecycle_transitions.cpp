#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "ros2_learning_lifecycle/sensor_node.hpp"
#include "ros2_learning_lifecycle/processor_node.hpp"

using LS = lifecycle_msgs::msg::State;
using LT = lifecycle_msgs::msg::Transition;

// ──────────────────────────────────────────────────────────────
// 测试夹具
// ──────────────────────────────────────────────────────────────
class LifecycleTransitionTest : public ::testing::Test
{
protected:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }

    // 触发转换并返回新状态 id
    uint8_t do_transition(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node, uint8_t transition_id)
    {
        return node->trigger_transition(
            rclcpp_lifecycle::Transition(transition_id)).id();
    }
};

// ──────────────────────────────────────────────────────────────
// SensorNode 测试
// ──────────────────────────────────────────────────────────────

// 1. configure → Inactive
TEST_F(LifecycleTransitionTest, SensorNodeConfigureSuccess)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    EXPECT_EQ(node->get_current_state().id(), LS::PRIMARY_STATE_UNCONFIGURED);

    const uint8_t new_state = do_transition(node, LT::TRANSITION_CONFIGURE);
    EXPECT_EQ(new_state, LS::PRIMARY_STATE_INACTIVE);
}

// 2. 完整正向循环：configure → activate → deactivate → cleanup → Unconfigured
TEST_F(LifecycleTransitionTest, SensorNodeFullCycle)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();

    EXPECT_EQ(do_transition(node, LT::TRANSITION_CONFIGURE), LS::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(do_transition(node, LT::TRANSITION_ACTIVATE),  LS::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(do_transition(node, LT::TRANSITION_DEACTIVATE), LS::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(do_transition(node, LT::TRANSITION_CLEANUP),    LS::PRIMARY_STATE_UNCONFIGURED);
}

// 3. 从 Active 状态 shutdown → Finalized
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

// 4. publisher 在 configure 后存在但未激活
TEST_F(LifecycleTransitionTest, SensorPublisherInactiveAfterConfigure)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    do_transition(node, LT::TRANSITION_CONFIGURE);

    ASSERT_NE(node->get_publisher(), nullptr);
    EXPECT_FALSE(node->get_publisher()->is_activated());
}

// 5. publisher 在 activate 后处于激活状态
TEST_F(LifecycleTransitionTest, SensorPublisherActiveAfterActivate)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    do_transition(node, LT::TRANSITION_CONFIGURE);
    do_transition(node, LT::TRANSITION_ACTIVATE);

    ASSERT_NE(node->get_publisher(), nullptr);
    EXPECT_TRUE(node->get_publisher()->is_activated());
}

// 6. deactivate 后 publisher 停用（timer 已销毁）
TEST_F(LifecycleTransitionTest, SensorPublisherInactiveAfterDeactivate)
{
    auto node = std::make_shared<ros2_learning_lifecycle::SensorNode>();
    do_transition(node, LT::TRANSITION_CONFIGURE);
    do_transition(node, LT::TRANSITION_ACTIVATE);
    do_transition(node, LT::TRANSITION_DEACTIVATE);

    ASSERT_NE(node->get_publisher(), nullptr);
    EXPECT_FALSE(node->get_publisher()->is_activated());
}

// ──────────────────────────────────────────────────────────────
// ProcessorNode 测试
// ──────────────────────────────────────────────────────────────

// 7. configure_behavior="failure" → configure 失败，回到 Unconfigured
TEST_F(LifecycleTransitionTest, ProcessorNodeFailureOnConfigure)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("configure_behavior", "failure");
    auto node = std::make_shared<ros2_learning_lifecycle::ProcessorNode>(opts);

    // configure 失败：节点应回到 Unconfigured（非 Inactive）
    const uint8_t state_after = do_transition(node, LT::TRANSITION_CONFIGURE);
    EXPECT_EQ(state_after, LS::PRIMARY_STATE_UNCONFIGURED);
}

// 8. configure_behavior="error" → on_error 触发后回到 Unconfigured
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
