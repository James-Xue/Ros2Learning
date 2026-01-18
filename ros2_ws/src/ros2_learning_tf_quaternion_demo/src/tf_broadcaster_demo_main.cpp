// tf_broadcaster_demo_main.cpp
// TF 广播器演示节点 - 主入口

#include "ros2_learning_tf_quaternion_demo/tf_broadcaster_demo.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcasterDemo>());
    rclcpp::shutdown();
    return 0;
}
