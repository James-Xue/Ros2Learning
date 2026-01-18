// tf_listener_demo_main.cpp
// TF 监听器演示节点 - 主入口

#include "ros2_learning_tf_quaternion_demo/tf_listener_demo.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFListenerDemo>());
    rclcpp::shutdown();
    return 0;
}
