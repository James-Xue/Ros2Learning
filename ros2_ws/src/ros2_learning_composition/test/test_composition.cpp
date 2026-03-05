/**
 * @file   test_composition.cpp
 * @brief  ros2_learning_composition 包单元测试。
 *
 * 测试策略：
 *   - 每个测试用例独享 rclcpp::init/shutdown 周期
 *   - 使用 MultiThreadedExecutor 后台 spin 处理回调
 *   - 通过 promise/future + atomic once-flag 同步异步消息，避免重复 set_value
 *   - RAII ExecutorGuard 保证异常安全的线程清理
 *   - 验证组件实例化、消息发布/接收、生命周期转换、intra-process 通信链
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ros2_learning_composition/image_producer.hpp"
#include "ros2_learning_composition/image_processor.hpp"
#include "ros2_learning_composition/image_saver.hpp"

using namespace std::chrono_literals;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ── RAII 线程守卫 ─────────────────────────────────────────────────────────

/**
 * @brief RAII 守卫：析构时自动 cancel executor 并 join spin 线程。
 *
 * 确保即使 ASSERT_* 失败导致提前返回，spin 线程也能被正确清理，
 * 避免 std::terminate。
 */
class ExecutorGuard
{
public:
    /**
     * @brief 构造守卫并启动 spin 后台线程。
     * @param[in] executor 多线程执行器。
     */
    explicit ExecutorGuard(
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
    : executor_(std::move(executor))
    , thread_([this]() { executor_->spin(); })
    {
    }

    /// @brief 禁用拷贝。
    ExecutorGuard(const ExecutorGuard &) = delete;
    /// @brief 禁用赋值。
    ExecutorGuard & operator=(const ExecutorGuard &) = delete;

    /**
     * @brief 析构：cancel executor 并 join 线程。
     */
    ~ExecutorGuard()
    {
        executor_->cancel();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;  ///< 执行器。
    std::thread thread_;  ///< spin 后台线程。
};

// ── Fixture ──────────────────────────────────────────────────────────────

/**
 * @class CompositionTest
 * @brief 组件测试夹具，管理 rclcpp 初始化/关闭周期。
 */
class CompositionTest : public ::testing::Test
{
protected:
    /// @brief 初始化 rclcpp。
    void SetUp() override { rclcpp::init(0, nullptr); }

    /// @brief 关闭 rclcpp。
    void TearDown() override { rclcpp::shutdown(); }
};

// ── 测试用例 ──────────────────────────────────────────────────────────────

/**
 * @brief 验证 ImageProducer 可通过 NodeOptions 正常实例化。
 */
TEST_F(CompositionTest, ProducerCanBeInstantiated)
{
    auto node = std::make_shared<ros2_learning_composition::ImageProducer>(
        rclcpp::NodeOptions{});
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(std::string(node->get_name()), "image_producer");
}

/**
 * @brief 验证 ImageProcessor 可通过 NodeOptions 正常实例化。
 */
TEST_F(CompositionTest, ProcessorCanBeInstantiated)
{
    auto node = std::make_shared<ros2_learning_composition::ImageProcessor>(
        rclcpp::NodeOptions{});
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(std::string(node->get_name()), "image_processor");
}

/**
 * @brief 验证 ImageSaver 可通过 NodeOptions 正常实例化（LifecycleNode 组件）。
 */
TEST_F(CompositionTest, SaverCanBeInstantiated)
{
    auto node = std::make_shared<ros2_learning_composition::ImageSaver>(
        rclcpp::NodeOptions{});
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(std::string(node->get_name()), "image_saver");
}

/**
 * @brief 验证 ImageProducer 定时发布消息到 pipeline/raw。
 */
TEST_F(CompositionTest, ProducerPublishesMessages)
{
    auto producer = std::make_shared<ros2_learning_composition::ImageProducer>(
        rclcpp::NodeOptions{});

    // 辅助订阅节点
    auto sub_node = rclcpp::Node::make_shared("test_subscriber");
    auto promise_ptr = std::make_shared<std::promise<sensor_msgs::msg::Image::SharedPtr>>();
    auto future = promise_ptr->get_future();
    auto once = std::make_shared<std::atomic<bool>>(false);

    auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
        "pipeline/raw", 10,
        [promise_ptr, once](sensor_msgs::msg::Image::SharedPtr msg) {
            if (!once->exchange(true)) {
                promise_ptr->set_value(msg);
            }
        });

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(producer);
    executor->add_node(sub_node);
    ExecutorGuard guard(executor);

    // 等待消息
    ASSERT_EQ(future.wait_for(3s), std::future_status::ready)
        << "Producer 应在 3 秒内发布消息";
    auto msg = future.get();

    EXPECT_EQ(msg->encoding, "rgb8");
    EXPECT_EQ(msg->width, 64u);
    EXPECT_EQ(msg->height, 64u);
    EXPECT_EQ(msg->data.size(), 64u * 64u * 3u);
}

/**
 * @brief 验证 ImageProcessor 能接收消息并转发到 pipeline/processed。
 */
TEST_F(CompositionTest, ProcessorForwardsMessages)
{
    auto processor = std::make_shared<ros2_learning_composition::ImageProcessor>(
        rclcpp::NodeOptions{});

    // 手动发布一条消息到 pipeline/raw
    auto pub_node = rclcpp::Node::make_shared("test_publisher");
    auto publisher = pub_node->create_publisher<sensor_msgs::msg::Image>("pipeline/raw", 10);

    // 订阅 pipeline/processed
    auto sub_node = rclcpp::Node::make_shared("test_processed_sub");
    auto promise_ptr = std::make_shared<std::promise<sensor_msgs::msg::Image::SharedPtr>>();
    auto future = promise_ptr->get_future();
    auto once = std::make_shared<std::atomic<bool>>(false);

    auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
        "pipeline/processed", 10,
        [promise_ptr, once](sensor_msgs::msg::Image::SharedPtr msg) {
            if (!once->exchange(true)) {
                promise_ptr->set_value(msg);
            }
        });

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(processor);
    executor->add_node(pub_node);
    executor->add_node(sub_node);
    ExecutorGuard guard(executor);

    // 等待 graph 建立后发布测试消息
    std::this_thread::sleep_for(200ms);

    auto test_msg = sensor_msgs::msg::Image();
    test_msg.header.stamp = pub_node->now();
    test_msg.width = 2;
    test_msg.height = 1;
    test_msg.encoding = "rgb8";
    test_msg.step = 6;
    test_msg.data = {0x00, 0xFF, 0x80, 0x10, 0x20, 0x30};
    publisher->publish(test_msg);

    // 等待处理后的消息
    ASSERT_EQ(future.wait_for(3s), std::future_status::ready)
        << "Processor 应在 3 秒内转发消息";
    auto result = future.get();

    // 验证像素被按位取反
    EXPECT_EQ(result->data[0], static_cast<uint8_t>(~0x00));  // 0xFF
    EXPECT_EQ(result->data[1], static_cast<uint8_t>(~0xFF));  // 0x00
    EXPECT_EQ(result->data[2], static_cast<uint8_t>(~0x80));  // 0x7F
}

/**
 * @brief 验证 ImageSaver 生命周期状态转换正常。
 */
TEST_F(CompositionTest, SaverLifecycleTransitions)
{
    auto saver = std::make_shared<ros2_learning_composition::ImageSaver>(
        rclcpp::NodeOptions{});

    // Unconfigured -> Inactive (configure)
    auto ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "configure 后应为 Inactive";

    // Inactive -> Active (activate)
    ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    EXPECT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        << "activate 后应为 Active";

    // Active -> Inactive (deactivate)
    ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    EXPECT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "deactivate 后应为 Inactive";

    // Inactive -> Unconfigured (cleanup)
    ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    EXPECT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "cleanup 后应为 Unconfigured";
}

/**
 * @brief 验证 ImageSaver 在 Active 状态下正确累积帧统计。
 */
TEST_F(CompositionTest, SaverCountsFramesWhenActive)
{
    auto saver = std::make_shared<ros2_learning_composition::ImageSaver>(
        rclcpp::NodeOptions{});

    // 发布到 pipeline/processed
    auto pub_node = rclcpp::Node::make_shared("test_frame_pub");
    auto publisher = pub_node->create_publisher<sensor_msgs::msg::Image>(
        "pipeline/processed", 10);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(saver->get_node_base_interface());
    executor->add_node(pub_node);
    ExecutorGuard guard(executor);

    // configure + activate（断言状态转换成功）
    auto ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // 等待 graph 建立
    std::this_thread::sleep_for(200ms);

    // 发送 3 帧
    for (int i = 0; i < 3; ++i) {
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = pub_node->now();
        msg.width = 2;
        msg.height = 1;
        msg.encoding = "rgb8";
        msg.step = 6;
        msg.data = {0, 0, 0, 0, 0, 0};
        publisher->publish(msg);
        std::this_thread::sleep_for(50ms);
    }

    // 等待回调处理
    std::this_thread::sleep_for(300ms);

    EXPECT_GE(saver->get_frame_count(), 1u)
        << "Active 状态下应至少接收到 1 帧";
}

/**
 * @brief 验证完整流水线：Producer→Processor→Saver 端到端通信（intra-process 模式）。
 */
TEST_F(CompositionTest, FullPipelineEndToEnd)
{
    // 启用 intra-process 通信
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

    auto producer = std::make_shared<ros2_learning_composition::ImageProducer>(options);
    auto processor = std::make_shared<ros2_learning_composition::ImageProcessor>(options);
    auto saver = std::make_shared<ros2_learning_composition::ImageSaver>(options);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(producer);
    executor->add_node(processor);
    executor->add_node(saver->get_node_base_interface());
    ExecutorGuard guard(executor);

    // configure + activate saver（断言状态转换成功）
    auto ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // 等待流水线运行一段时间
    std::this_thread::sleep_for(1s);

    EXPECT_GE(saver->get_frame_count(), 1u)
        << "流水线应至少端到端传递 1 帧";
    EXPECT_GE(saver->get_total_latency(), 0.0)
        << "总延迟应为非负数";
}

/**
 * @brief 验证 ImageSaver 在非 Active 状态下不计数。
 */
TEST_F(CompositionTest, SaverIgnoresFramesWhenInactive)
{
    auto saver = std::make_shared<ros2_learning_composition::ImageSaver>(
        rclcpp::NodeOptions{});

    // 仅 configure，不 activate（断言 configure 成功）
    auto ret = saver->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(ret.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    auto pub_node = rclcpp::Node::make_shared("test_inactive_pub");
    auto publisher = pub_node->create_publisher<sensor_msgs::msg::Image>(
        "pipeline/processed", 10);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(saver->get_node_base_interface());
    executor->add_node(pub_node);
    ExecutorGuard guard(executor);

    std::this_thread::sleep_for(200ms);

    // 发送消息
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = pub_node->now();
    msg.width = 2;
    msg.height = 1;
    msg.encoding = "rgb8";
    msg.step = 6;
    msg.data = {0, 0, 0, 0, 0, 0};
    publisher->publish(msg);

    std::this_thread::sleep_for(300ms);

    EXPECT_EQ(saver->get_frame_count(), 0u)
        << "Inactive 状态下不应计数";
}

/**
 * @brief 验证 ImageProducer 非法参数校验（width=0 应抛异常）。
 */
TEST_F(CompositionTest, ProducerRejectsInvalidParams)
{
    // image_width=0 应抛 rclcpp::exceptions::InvalidParameterValueException
    auto options = rclcpp::NodeOptions{}.append_parameter_override("image_width", 0);
    EXPECT_THROW(
        std::make_shared<ros2_learning_composition::ImageProducer>(options),
        rclcpp::exceptions::InvalidParameterValueException);
}

/**
 * @brief 测试入口。
 * @param[in] argc 命令行参数数量。
 * @param[in] argv 命令行参数数组。
 * @return 测试结果（0=全部通过）。
 */
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
