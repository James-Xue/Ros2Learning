// main.cpp
// 系统信息发布器主入口

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_learning_sysinfo_publisher/sysinfo_publisher_node.hpp"

/**
 * @brief 主函数
 * 
 * 创建并运行系统信息发布器节点
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    using ros2_learning_sysinfo_publisher::SysInfoPublisherNode;
    rclcpp::spin(std::make_shared<SysInfoPublisherNode>());
    
    rclcpp::shutdown();
    return 0;
}
