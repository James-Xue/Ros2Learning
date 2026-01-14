// 抓取/放置占位节点实现：提供 Trigger 服务并用 sleep 模拟耗时
#include "manipulation_stub.hpp"

#include <chrono>

using namespace std::chrono_literals;

// 构造函数：声明参数并创建服务
ManipulationStub::ManipulationStub()
        : Node("manipulation_stub")
{
    // 模拟抓取/放置耗时（秒）
    operation_time_sec_ =
        this->declare_parameter<double>("operation_time_sec", 1.0);

    // 提供 /manipulation/pick 服务
    pick_service_ = this->create_service<Trigger>(
        "/manipulation/pick",
        [this](const std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response)
        { handle_pick(request, response); });

    // 提供 /manipulation/place 服务
    place_service_ = this->create_service<Trigger>(
        "/manipulation/place",
        [this](const std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response)
        { handle_place(request, response); });
}

void ManipulationStub::handle_pick(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
{
    // Trigger 请求不带参数，忽略即可
    (void)request;
    RCLCPP_INFO(get_logger(), "Pick requested, simulating for %.1f seconds.",
                operation_time_sec_);
    // 将秒数转换为纳秒，避免浮点精度问题
    const auto sleep_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(operation_time_sec_));
    // 阻塞等待，模拟抓取动作
    rclcpp::sleep_for(sleep_duration);
    // 返回成功结果
    response->success = true;
    response->message = "pick ok";
}

void ManipulationStub::handle_place(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
{
    // Trigger 请求不带参数，忽略即可
    (void)request;
    RCLCPP_INFO(get_logger(), "Place requested, simulating for %.1f seconds.",
                operation_time_sec_);
    // 将秒数转换为纳秒，避免浮点精度问题
    const auto sleep_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(operation_time_sec_));
    // 阻塞等待，模拟放置动作
    rclcpp::sleep_for(sleep_duration);
    // 返回成功结果
    response->success = true;
    response->message = "place ok";
}
