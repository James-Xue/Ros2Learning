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
    
    // 初始化夹爪控制接口
    if (!node->initializeGripper()) {
        RCLCPP_ERROR(node->get_logger(), "夹爪初始化失败");
        return 1;
    }
    
    // 运行基础演示
    RCLCPP_INFO(node->get_logger(), "========== 基础演示 ==========");
    node->runDemo();
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 运行正方形演示
    RCLCPP_INFO(node->get_logger(), "\n========== 笛卡尔路径演示 ==========");
    node->drawSquare();
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 运行抓取和放置演示
    RCLCPP_INFO(node->get_logger(), "\n========== 夹爪控制演示 ==========");
    node->runPickAndPlaceDemo();
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 运行真实物体抓取演示
    RCLCPP_INFO(node->get_logger(), "\n========== 真实物体抓取演示 ==========");
    node->runRealisticPickAndPlace();
    
    // 演示完成，退出（不需要spin）
    RCLCPP_INFO(node->get_logger(), "\n所有演示完成，程序退出");
    // rclcpp::spin(node);  // 去掉spin，演示后自动退出
    
    rclcpp::shutdown();
    return 0;
}
