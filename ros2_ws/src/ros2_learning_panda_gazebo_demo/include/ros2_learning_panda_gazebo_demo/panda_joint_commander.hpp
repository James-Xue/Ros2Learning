#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <string>
#include <vector>

namespace ros2_learning_panda_gazebo_demo
{

class PandaJointCommander : public rclcpp::Node
{
public:
    PandaJointCommander();

private:
    void onTimer();

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
    rclcpp::Time start_time_;
};

}  // namespace ros2_learning_panda_gazebo_demo
