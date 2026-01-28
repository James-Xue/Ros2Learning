#include "ros2_learning_panda_gazebo_demo/panda_joint_commander.hpp"

#include <cmath>

namespace ros2_learning_panda_gazebo_demo
{

PandaJointCommander::PandaJointCommander()
: Node("panda_joint_commander"), start_time_(this->now())
{
    const std::array<std::string, 7> topics = {
        "/panda/joint1/cmd_pos",
        "/panda/joint2/cmd_pos",
        "/panda/joint3/cmd_pos",
        "/panda/joint4/cmd_pos",
        "/panda/joint5/cmd_pos",
        "/panda/joint6/cmd_pos",
        "/panda/joint7/cmd_pos",
    };

    publishers_.reserve(topics.size());
    for (const auto & topic : topics) {
        publishers_.push_back(this->create_publisher<std_msgs::msg::Float64>(topic, 10));
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PandaJointCommander::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Panda joint commander started.");
}

void PandaJointCommander::onTimer()
{
    const double t = (this->now() - start_time_).seconds();
    const std::array<double, 7> phase = {0.0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8};
    const std::array<double, 7> lower = {-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671};
    const std::array<double, 7> upper = { 2.9671,  1.8326,  2.9671,  0.0873,  2.9671,  3.8223,  2.9671};

    for (size_t i = 0; i < publishers_.size(); ++i) {
        const double mid = (lower[i] + upper[i]) * 0.5;
        const double amplitude = (upper[i] - lower[i]) * 0.3;
        std_msgs::msg::Float64 cmd;
        cmd.data = mid + amplitude * std::sin(t + phase[i]);
        if (cmd.data > upper[i]) {
            cmd.data = upper[i];
        } else if (cmd.data < lower[i]) {
            cmd.data = lower[i];
        }
        publishers_[i]->publish(cmd);
    }
}

}  // namespace ros2_learning_panda_gazebo_demo
