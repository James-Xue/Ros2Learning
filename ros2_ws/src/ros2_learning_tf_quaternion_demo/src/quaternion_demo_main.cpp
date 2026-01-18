// quaternion_demo_main.cpp
// 四元数演示节点 - 主入口

#include "ros2_learning_tf_quaternion_demo/quaternion_demo.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuaternionDemo>());
    rclcpp::shutdown();
    return 0;
}
