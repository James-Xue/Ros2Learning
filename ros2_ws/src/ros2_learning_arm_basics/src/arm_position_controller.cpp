// arm_position_controller.cpp
// 机械臂位置控制器类的实现文件
#include "ros2_learning_arm_basics/arm_position_controller.hpp"

#include <chrono>
#include <string>

/**
 * @brief 构造函数
 * 
 * 初始化节点和MoveGroup接口
 * PLANNING_GROUP 是在SRDF中定义的规划组名称
 */
ArmPositionController::ArmPositionController() 
    : Node("arm_position_controller"),
      m_logger(this->get_logger())
{
    RCLCPP_INFO(m_logger, "机械臂位置控制节点正在初始化...");
    
    // 注意：MoveGroupInterface需要在node初始化后才能创建
    // 这里我们在单独的初始化函数中完成
}

/**
 * @brief 初始化MoveGroup接口
 * 
 * @param planning_group 规划组名称（如"panda_arm"）
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ArmPositionController::initialize(const std::string& planning_group) {
    try {
        // 创建MoveGroup接口
        // 这是与MoveIt规划器交互的主要接口
        m_moveGroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group);
        
        RCLCPP_INFO(m_logger, "规划组: %s", planning_group.c_str());
        RCLCPP_INFO(m_logger, "规划框架: %s", m_moveGroup->getPlanningFrame().c_str());
        RCLCPP_INFO(m_logger, "末端执行器: %s", m_moveGroup->getEndEffectorLink().c_str());
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(m_logger, "初始化失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 移动到预定义姿态
 * 
 * 预定义姿态在SRDF文件中配置（如"ready", "home"等）
 * 
 * @param target_name 目标姿态名称
 */
void ArmPositionController::moveToNamedTarget(const std::string& target_name) {
    RCLCPP_INFO(m_logger, "正在移动到预定义姿态: %s", target_name.c_str());
    
    // 设置目标为预定义姿态
    m_moveGroup->setNamedTarget(target_name);
    
    // 规划并执行运动
    // move()函数会自动调用plan()和execute()
    auto success = (m_moveGroup->move() == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "成功到达姿态: %s", target_name.c_str());
    } else {
        RCLCPP_WARN(m_logger, "移动到姿态 %s 失败", target_name.c_str());
    }
}

/**
 * @brief 移动到指定的笛卡尔空间位置
 * 
 * 使用逆运动学计算关节角度，然后规划路径
 * 
 * @param target_pose 目标位姿（位置+方向）
 */
void ArmPositionController::moveToPose(const geometry_msgs::msg::Pose& target_pose) {
    RCLCPP_INFO(m_logger, "正在移动到笛卡尔空间目标位置");
    RCLCPP_INFO(m_logger, "  位置: [%.3f, %.3f, %.3f]", 
                target_pose.position.x, 
                target_pose.position.y, 
                target_pose.position.z);
    
    // 设置目标位姿
    m_moveGroup->setPoseTarget(target_pose);
    
    // 规划运动路径
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (m_moveGroup->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "规划成功，正在执行...");
        m_moveGroup->execute(plan);
    } else {
        RCLCPP_WARN(m_logger, "规划失败");
    }
}

/**
 * @brief 直接控制关节角度
 * 
 * 在关节空间中规划和执行运动
 * 
 * @param joint_values 目标关节角度值（单位：弧度）
 */
void ArmPositionController::moveJoints(const std::vector<double>& joint_values) {
    RCLCPP_INFO(m_logger, "正在移动到指定关节角度");
    
    // 打印目标关节角度
    std::string joint_str = "[";
    for(size_t i = 0; i < joint_values.size(); ++i) {
        joint_str += std::to_string(joint_values[i]);
        if(i < joint_values.size() - 1) joint_str += ", ";
    }
    joint_str += "]";
    RCLCPP_INFO(m_logger, "  关节角度: %s", joint_str.c_str());
    
    // 设置目标关节值
    m_moveGroup->setJointValueTarget(joint_values);
    
    // 执行规划和运动
    auto success = (m_moveGroup->move() == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "成功到达目标关节角度");
    } else {
        RCLCPP_WARN(m_logger, "移动失败");
    }
}

/**
 * @brief 运行演示序列
 * 
 * 展示基本的机械臂控制功能
 */
void ArmPositionController::runDemo() {
    RCLCPP_INFO(m_logger, "\n========================================");
    RCLCPP_INFO(m_logger, "  开始机械臂控制演示");
    RCLCPP_INFO(m_logger, "========================================\n");
    
    // 演示1: 移动到ready姿态
    RCLCPP_INFO(m_logger, "[演示1] 移动到ready姿态");
    moveToNamedTarget("ready");
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 演示2: 移动到指定位置
    RCLCPP_INFO(m_logger, "\n[演示2] 移动到自定义笛卡尔位置");
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;  // 四元数单位方向
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;
    moveToPose(target_pose);
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 演示3: 回到home姿态
    RCLCPP_INFO(m_logger, "\n[演示3] 返回home姿态");
    moveToNamedTarget("ready");
    
    RCLCPP_INFO(m_logger, "\n========================================");
    RCLCPP_INFO(m_logger, "  演示完成！");
    RCLCPP_INFO(m_logger, "========================================\n");
}
