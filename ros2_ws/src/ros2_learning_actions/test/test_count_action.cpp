/**
 * @file test_count_action.cpp
 * @brief CountActionServer 单元测试。
 *
 * 测试策略：
 *   - Fixture 启动 MultiThreadedExecutor 后台线程，server 和匿名 client 同时 spin
 *   - 匿名 client 直接用 rclcpp_action::create_client，不依赖 CountActionClient
 *   - 所有测试通过 shared_ptr<promise>/future 同步，避免 sleep 轮询
 *   - 每个用例独享 rclcpp::init/shutdown 周期（Fixture SetUp/TearDown）
 */

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_learning_actions/count_action_server.hpp"

using namespace std::chrono_literals;
using CountTask = ros2_learning_custom_interfaces::action::CountTask;
using GoalHandleClient = rclcpp_action::ClientGoalHandle<CountTask>;

/**
 * @class CountActionTest
 * @brief CountActionServer 行为测试夹具。
 *
 * 架构：
 *   - server_node_    : CountActionServer（被测对象）
 *   - client_node_    : 普通 rclcpp::Node（承载匿名 action client）
 *   - action_client_  : rclcpp_action::Client（测试驱动器）
 *   - executor_       : MultiThreadedExecutor（同时 spin 两个节点）
 *   - executor_thread_: 后台线程，持续 spin executor
 */
class CountActionTest : public ::testing::Test
{
protected:
    /// @brief 初始化 rclcpp，创建 server/client 节点，启动 executor 后台线程。
    void SetUp() override
    {
        rclcpp::init(0, nullptr);

        // 创建被测 server 节点
        server_node_ = std::make_shared<ros2_learning_actions::CountActionServer>();

        // 创建承载匿名 client 的辅助节点
        client_node_ = rclcpp::Node::make_shared("test_client_node");

        // 创建 action client
        action_client_ = rclcpp_action::create_client<CountTask>(
            client_node_, "count_task");

        // MultiThreadedExecutor：server 的 execute 线程调用 publish_feedback 等
        // 需要 executor 处理 waitable，单线程会死锁
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(server_node_);
        executor_->add_node(client_node_);

        executor_thread_ = std::thread([this]() { executor_->spin(); });

        // 等待 server 注册到 ROS graph
        ASSERT_TRUE(action_client_->wait_for_action_server(5s))
            << "Action server 未在 5 秒内就绪";
    }

    /// @brief 停止 executor 线程，关闭 rclcpp。
    void TearDown() override
    {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        rclcpp::shutdown();
    }

    /**
     * @brief 发送 goal 并等待 goal_response（接受/拒绝）。
     * @param[in] target     计数目标。
     * @param[in] period_sec 每次计数等待时间。
     * @param[in] timeout    等待超时。
     * @return goal_handle（nullptr 表示被拒绝）。
     */
    GoalHandleClient::SharedPtr send_goal_sync(
        int32_t target, float period_sec,
        std::chrono::seconds timeout = 5s)
    {
        // 用 shared_ptr<promise> 避免 lambda 超时后访问已销毁的栈变量
        auto promise_ptr =
            std::make_shared<std::promise<GoalHandleClient::SharedPtr>>();
        auto future = promise_ptr->get_future();

        CountTask::Goal goal;
        goal.target = target;
        goal.period_sec = period_sec;

        auto options = rclcpp_action::Client<CountTask>::SendGoalOptions();
        options.goal_response_callback =
            [promise_ptr](const GoalHandleClient::SharedPtr & handle) {
                promise_ptr->set_value(handle);
            };

        action_client_->async_send_goal(goal, options);

        if (future.wait_for(timeout) != std::future_status::ready) {
            return nullptr;
        }
        return future.get();
    }

    /**
     * @brief 发送 goal 并等待最终 Result。
     * @param[in] target      计数目标。
     * @param[in] period_sec  每次计数等待时间。
     * @param[in] feedback_cb 可选 feedback 回调。
     * @param[in] timeout     等待超时。
     * @return 封装结果（包含 ResultCode 和 Result 字段）。
     */
    GoalHandleClient::WrappedResult send_goal_and_wait_result(
        int32_t target, float period_sec,
        std::function<void(GoalHandleClient::SharedPtr,
            const std::shared_ptr<const CountTask::Feedback>)> feedback_cb = nullptr,
        std::chrono::seconds timeout = 10s)
    {
        auto result_promise =
            std::make_shared<std::promise<GoalHandleClient::WrappedResult>>();
        auto result_future = result_promise->get_future();

        CountTask::Goal goal;
        goal.target = target;
        goal.period_sec = period_sec;

        auto options = rclcpp_action::Client<CountTask>::SendGoalOptions();
        if (feedback_cb) {
            options.feedback_callback = feedback_cb;
        }
        options.result_callback =
            [result_promise](const GoalHandleClient::WrappedResult & result) {
                result_promise->set_value(result);
            };

        action_client_->async_send_goal(goal, options);

        if (result_future.wait_for(timeout) != std::future_status::ready) {
            ADD_FAILURE() << "等待 Result 超时（" << timeout.count() << "s）";
            return GoalHandleClient::WrappedResult{};
        }
        return result_future.get();
    }

    std::shared_ptr<ros2_learning_actions::CountActionServer> server_node_;  ///< 被测节点。
    rclcpp::Node::SharedPtr client_node_;  ///< 承载匿名 client 的辅助节点。
    rclcpp_action::Client<CountTask>::SharedPtr action_client_;  ///< 测试驱动用 client。
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;  ///< 多线程执行器。
    std::thread executor_thread_;  ///< executor spin 后台线程。
};

// ── 测试用例 ──────────────────────────────────────────────────────────────

/**
 * @brief 验证合法参数目标被接受（ACCEPT_AND_EXECUTE）并正常完成。
 */
TEST_F(CountActionTest, ServerAcceptsValidGoal)
{
    // 同时验证接受和完成，避免 execute 线程在 TearDown 后仍运行
    auto result = send_goal_and_wait_result(2, 0.05f);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
        << "合法目标（target=2, period=0.05）应被 server 接受并完成";
}

/**
 * @brief 验证 target <= 0 被拒绝（REJECT）。
 */
TEST_F(CountActionTest, ServerRejectsZeroTarget)
{
    auto goal_handle = send_goal_sync(0, 0.1f);
    EXPECT_EQ(goal_handle, nullptr)
        << "target=0 应被 REJECT";
}

/**
 * @brief 验证 period_sec == 0 被拒绝（REJECT）。
 */
TEST_F(CountActionTest, ServerRejectsZeroPeriod)
{
    auto goal_handle = send_goal_sync(5, 0.0f);
    EXPECT_EQ(goal_handle, nullptr)
        << "period_sec=0 应被 REJECT";
}

/**
 * @brief 验证 period_sec < 0 被拒绝（REJECT）。
 */
TEST_F(CountActionTest, ServerRejectsNegativePeriod)
{
    auto goal_handle = send_goal_sync(5, -0.5f);
    EXPECT_EQ(goal_handle, nullptr)
        << "period_sec=-0.5 应被 REJECT";
}

/**
 * @brief 验证 target 为负数被拒绝（REJECT）。
 */
TEST_F(CountActionTest, ServerRejectsNegativeTarget)
{
    auto goal_handle = send_goal_sync(-1, 0.1f);
    EXPECT_EQ(goal_handle, nullptr)
        << "target=-1 应被 REJECT";
}

/**
 * @brief 验证正常完成：final_count == target 且 succeeded == true。
 */
TEST_F(CountActionTest, ServerSucceedsWithCorrectResult)
{
    // 极小 period（0.05s）让 3 次计数约 0.15s 完成
    auto result = send_goal_and_wait_result(3, 0.05f);

    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
        << "action 应以 SUCCEEDED 完成";
    ASSERT_NE(result.result, nullptr) << "result 不得为空";
    EXPECT_EQ(result.result->final_count, 3)
        << "final_count 应等于 target";
    EXPECT_TRUE(result.result->succeeded)
        << "succeeded 字段应为 true";
    EXPECT_GT(result.result->elapsed_sec, 0.0f)
        << "elapsed_sec 应大于 0";
}

/**
 * @brief 验证取消路径：发送 goal 后立即取消，result.succeeded == false。
 */
TEST_F(CountActionTest, ServerHandlesCancelRequest)
{
    // 目标设大（100 次，每次 0.5s），确保在取消前不会自然完成
    auto handle_promise =
        std::make_shared<std::promise<GoalHandleClient::SharedPtr>>();
    auto result_promise =
        std::make_shared<std::promise<GoalHandleClient::WrappedResult>>();

    CountTask::Goal goal;
    goal.target = 100;
    goal.period_sec = 0.5f;

    auto options = rclcpp_action::Client<CountTask>::SendGoalOptions();
    options.goal_response_callback =
        [handle_promise](const GoalHandleClient::SharedPtr & handle) {
            handle_promise->set_value(handle);
        };
    options.result_callback =
        [result_promise](const GoalHandleClient::WrappedResult & result) {
            result_promise->set_value(result);
        };

    action_client_->async_send_goal(goal, options);

    // 等待 goal 被接受
    auto handle_future = handle_promise->get_future();
    ASSERT_EQ(handle_future.wait_for(5s), std::future_status::ready)
        << "goal 应被接受";
    auto goal_handle = handle_future.get();
    ASSERT_NE(goal_handle, nullptr) << "goal_handle 不得为 nullptr";

    // 接受后立即发送取消请求
    action_client_->async_cancel_goal(goal_handle);

    // 等待 result
    auto result_future = result_promise->get_future();
    ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready)
        << "取消后应在 5s 内收到 result";
    auto result = result_future.get();

    EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED)
        << "ResultCode 应为 CANCELED";
    ASSERT_NE(result.result, nullptr);
    EXPECT_FALSE(result.result->succeeded)
        << "取消后 succeeded 应为 false";
}

/**
 * @brief 验证 Feedback 字段正确（current 递增，percent 在 [0, 100]）。
 */
TEST_F(CountActionTest, FeedbackFieldsAreCorrect)
{
    std::vector<int32_t> received_currents;
    std::mutex feedback_mutex;

    auto feedback_cb = [&received_currents, &feedback_mutex](
        GoalHandleClient::SharedPtr /*goal_handle*/,
        const std::shared_ptr<const CountTask::Feedback> feedback)
    {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        received_currents.push_back(feedback->current);
        // percent 必须在合法范围内
        EXPECT_GE(feedback->percent, 0.0f);
        EXPECT_LE(feedback->percent, 100.0f);
    };

    // target=5，期待多次 feedback
    auto result = send_goal_and_wait_result(5, 0.05f, feedback_cb);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

    std::lock_guard<std::mutex> lock(feedback_mutex);
    EXPECT_GE(received_currents.size(), 1u)
        << "至少应收到 1 次 feedback";
    // 验证 current 单调递增
    for (size_t i = 1; i < received_currents.size(); ++i) {
        EXPECT_GT(received_currents[i], received_currents[i - 1])
            << "current 应单调递增";
    }
}

/**
 * @brief 测试入口：初始化 GTest 并运行所有测试用例。
 * @param[in] argc 命令行参数数量。
 * @param[in] argv 命令行参数数组。
 * @return 测试结果（0=全部通过）。
 */
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
