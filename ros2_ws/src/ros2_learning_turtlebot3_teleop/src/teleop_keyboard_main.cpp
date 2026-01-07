// teleop_keyboard_main.cpp
// - 将 main 从 teleop_keyboard.cpp 中抽出，方便学习/调试。

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "teleop_keyboard_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
