#include "manipulation_stub.hpp"

#include <chrono>

using namespace std::chrono_literals;

ManipulationStub::ManipulationStub()
        : Node("manipulation_stub")
{
    operation_time_sec_ = this->declare_parameter<double>("operation_time_sec", 1.0);

    pick_service_ = this->create_service<Trigger>(
        "/manipulation/pick",
        [this](const std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response)
        { handle_pick(request, response); });

    place_service_ = this->create_service<Trigger>(
        "/manipulation/place",
        [this](const std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response)
        { handle_place(request, response); });
}

void ManipulationStub::handle_pick(const std::shared_ptr<Trigger::Request> request,
                                   std::shared_ptr<Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(get_logger(), "Pick requested, simulating for %.1f seconds.", operation_time_sec_);
    const auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(operation_time_sec_));
    rclcpp::sleep_for(sleep_duration);
    response->success = true;
    response->message = "pick ok";
}

void ManipulationStub::handle_place(const std::shared_ptr<Trigger::Request> request,
                                    std::shared_ptr<Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(get_logger(), "Place requested, simulating for %.1f seconds.", operation_time_sec_);
    const auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(operation_time_sec_));
    rclcpp::sleep_for(sleep_duration);
    response->success = true;
    response->message = "place ok";
}
