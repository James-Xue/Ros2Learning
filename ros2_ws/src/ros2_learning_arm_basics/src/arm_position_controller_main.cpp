// arm_position_controller_main.cpp
// 机械臂位置控制器的主函数入口
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros2_learning_arm_basics/arm_position_controller.hpp"

/**
 * @brief 主函数
 * 
 * 程序入口点，负责：
 * 1. 初始化ROS 2
 * 2. 创建控制节点
 * 3. 运行演示
 * 4. 保持节点运行
 */
int main(int argc, char** argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<ArmPositionController>();
    
    // 初始化MoveGroup（使用panda_arm规划组）
    if (!node->initialize("panda_arm")) {
        RCLCPP_ERROR(node->get_logger(), "节点初始化失败");
        return 1;
    }
    
    // 运行演示
    node->runDemo();
    
    // 保持节点运行以便接收回调
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
