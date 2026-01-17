// arm_position_controller.hpp
// 机械臂位置控制器类的头文件声明
#ifndef ROS2_LEARNING_ARM_BASICS_ARM_POSITION_CONTROLLER_HPP_
#define ROS2_LEARNING_ARM_BASICS_ARM_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

/**
 * @class ArmPositionController
 * @brief 机械臂位置控制器类 - 用于学习MoveIt 2基础
 * 
 * 这个节点展示了如何使用MoveIt 2控制机械臂：
 * - 移动到预定义姿态（如"ready", "home"）
 * - 移动到指定的笛卡尔空间位置
 * - 直接控制关节角度
 */
class ArmPositionController : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * 
     * 初始化节点，创建logger
     */
    ArmPositionController();
    
    /**
     * @brief 初始化MoveGroup接口
     * 
     * @param planning_group 规划组名称（如"panda_arm"）
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool initialize(const std::string& planning_group);
    
    /**
     * @brief 移动到预定义姿态
     * 
     * 预定义姿态在SRDF文件中配置（如"ready", "home"等）
     * 
     * @param target_name 目标姿态名称
     */
    void moveToNamedTarget(const std::string& target_name);
    
    /**
     * @brief 移动到指定的笛卡尔空间位置
     * 
     * 使用逆运动学计算关节角度，然后规划路径
     * 
     * @param target_pose 目标位姿（位置+方向）
     */
    void moveToPose(const geometry_msgs::msg::Pose& target_pose);
    
    /**
     * @brief 直接控制关节角度
     * 
     * 在关节空间中规划和执行运动
     * 
     * @param joint_values 目标关节角度值（单位：弧度）
     */
    void moveJoints(const std::vector<double>& joint_values);
    
    /**
     * @brief 运行演示序列
     * 
     * 展示基本的机械臂控制功能
     */
    void runDemo();
    
    /**
     * @brief 画正方形演示
     * 
     * 使用笛卡尔路径规划让末端沿正方形路径运动
     */
    void drawSquare();

private:
    /// MoveIt规划接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_moveGroup;
    
    /// 日志记录器
    rclcpp::Logger m_logger;
};

#endif  // ROS2_LEARNING_ARM_BASICS_ARM_POSITION_CONTROLLER_HPP_
