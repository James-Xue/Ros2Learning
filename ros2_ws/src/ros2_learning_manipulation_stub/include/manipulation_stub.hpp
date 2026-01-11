#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ManipulationStub : public rclcpp::Node
{
  public:
    using Trigger = std_srvs::srv::Trigger;

    ManipulationStub();

  private:
    void handle_pick(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);
    void handle_place(const std::shared_ptr<Trigger::Request> request,
                      std::shared_ptr<Trigger::Response> response);

    double operation_time_sec_{1.0};

    rclcpp::Service<Trigger>::SharedPtr pick_service_;
    rclcpp::Service<Trigger>::SharedPtr place_service_;
};
