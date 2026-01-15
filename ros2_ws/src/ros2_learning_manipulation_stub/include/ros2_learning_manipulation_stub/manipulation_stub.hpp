#pragma once

// 抓取/放置占位节点定义：提供 Trigger 服务用于学习流程编排
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ManipulationStub : public rclcpp::Node
{
  public:
    // 常用类型别名
    using Trigger = std_srvs::srv::Trigger;

    // 构造函数
    ManipulationStub();

  private:
    // 服务回调：抓取/放置
    void handle_pick(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);
    void handle_place(const std::shared_ptr<Trigger::Request> request,
                      std::shared_ptr<Trigger::Response> response);

    // 模拟动作耗时（秒）
    double operation_time_sec_{1.0};

    // ROS 服务句柄
    rclcpp::Service<Trigger>::SharedPtr pick_service_;
    rclcpp::Service<Trigger>::SharedPtr place_service_;
};
