#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <string>
#include <vector>

namespace ros2_learning_panda_gazebo_demo
{

// PandaJointCommander 类继承自 rclcpp::Node，是一个标准的 ROS 2 节点。
// 它的主要功能是周期性地发布命令，控制 Panda 机械臂的 7 个关节进行正弦波运动。
class PandaJointCommander : public rclcpp::Node
{
public:
    // 构造函数：初始化节点、发布者和定时器。
    PandaJointCommander();

private:
    // 定时器回调函数：计算正弦波轨迹并发布控制命令。
    // 这是核心业务逻辑所在，控制机械臂如何持续运动。
    void onTimer();

private:
    // 定时器指针：用于周期性触发回调。
    rclcpp::TimerBase::SharedPtr timer_;

    // 使用别名简化长类型
    using Float64Msg = std_msgs::msg::Float64;
    
    // 发布者列表：因为有 7 个关节，我们使用一个 vector 来存储 7 个发布者。
    // 每个发布者对应一个关节的控制话题。
    std::vector<rclcpp::Publisher<Float64Msg>::SharedPtr> publishers_;
    
    // 记录节点的启动时间：用于在计算正弦波时作为时间基准 (t)。
    rclcpp::Time start_time_;
};

}  // namespace ros2_learning_panda_gazebo_demo
