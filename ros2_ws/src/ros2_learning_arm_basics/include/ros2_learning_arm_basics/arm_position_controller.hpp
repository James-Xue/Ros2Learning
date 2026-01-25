// arm_position_controller.hpp
// 机械臂位置控制器类的头文件声明
#ifndef ROS2_LEARNING_ARM_BASICS_ARM_POSITION_CONTROLLER_HPP_
#define ROS2_LEARNING_ARM_BASICS_ARM_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

// ═══════════════════════════════════════
// 类型别名 - 简化长类型名
// ═══════════════════════════════════════
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using Pose = geometry_msgs::msg::Pose;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using Trajectory = moveit_msgs::msg::RobotTrajectory;
using ErrorCode = moveit::core::MoveItErrorCode;
using PlanningScenePublisher = rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr;

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
    bool moveToPose(const Pose& target_pose);
    
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
    
    // ═════════════════════════════════════════
    // 夹爪控制方法
    // ═════════════════════════════════════════
    
    /**
     * @brief 初始化夹爪控制接口
     * 
     * 创建"hand"规划组的MoveGroup接口
     * 
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool initializeGripper();
    
    /**
     * @brief 打开夹爪
     * 
     * 移动到预定义的"open"姿态（3.5cm宽度）
     */
    bool openGripper();
    
    /**
     * @brief 闭合夹爪
     * 
     * 移动到预定义的"close"姿态（完全闭合）
     */
    bool closeGripper();
    
    /**
     * @brief 设置夹爪宽度
     * 
     * @param width 夹爪宽度（单位：米，范围：0.0 ~ 0.08）
     */
    bool setGripperWidth(double width);
    

    
    // ═════════════════════════════════════════
    // 物体管理和真实抓取方法
    // ═════════════════════════════════════════
    
    /**
     * @brief 在场景中生成目标物体
     * 
     * 在机械臂前方(0.4, 0.0, 0.025)位置生成5cm立方体
     */
    void spawnTargetObject();
    
    /**
     * @brief 从场景中移除目标物体
     */
    void removeTargetObject();
    
    /**
     * @brief 将物体附加到夹爪
     * 
     * 实现物理附加，物体将跟随夹爪移动
     * 
     * @param object_id 物体ID
     */
    void attachObjectToGripper(const std::string& object_id);
    
    /**
     * @brief 从夹爪分离物体
     * 
     * 物体将留在当前位置
     * 
     * @param object_id 物体ID
     */
    void detachObjectFromGripper(const std::string& object_id);
    
    /**
     * @brief 真实的抓取和放置演示
     * 
     * 包含物体生成、附加、分离的完整流程
     */
    void runRealisticPickAndPlace();
    
    /**
     * @brief 允许/禁止夹爪与指定物体碰撞
     * 
     * 通过发布 PlanningScene 消息修改 ACM (Allowed Collision Matrix)
     * 
     * @param object_id 物体ID
     * @param allow true=允许碰撞, false=禁止碰撞
     */
    void allowObjectCollision(const std::string& object_id, bool allow);

private:
    // ═════════════════════════════════════════
    // 辅助函数（碰撞物体创建）
    // ═════════════════════════════════════════
    
    /**
     * @brief 创建桌面碰撞物体
     * 
     * @return 桌面碰撞物体消息
     */
    moveit_msgs::msg::CollisionObject createTableCollisionObject();
    
    /**
     * @brief 创建目标物体碰撞物体
     * 
     * @return 目标物体碰撞物体消息
     */
    moveit_msgs::msg::CollisionObject createTargetBoxCollisionObject();
    
    // ═════════════════════════════════════════
    // Pick-and-Place 辅助函数
    // ═════════════════════════════════════════
    
    /**
     * @brief 移动到物体上方的准备位置
     * 
     * @param object_id 目标物体ID
     * @return true 成功
     * @return false 失败
     */
    bool moveToPreGraspPosition(const std::string& object_id);
    
    /**
     * @brief 抓取指定物体
     * 
     * @param object_id 目标物体ID
     * @return true 成功抓取
     * @return false 抓取失败
     */
    bool graspObject(const std::string& object_id);
    
    /**
     * @brief 将物体放置到目标位置
     * 
     * @param object_id 目标物体ID
     * @return true 成功放置
     * @return false 放置失败
     */
    bool placeObject(const std::string& object_id);
    
    /**
     * @brief 清理场景并返回初始位置
     * 
     * @param object_id 要移除的物体ID
     */
    void cleanupAndReturnHome(const std::string& object_id);

private:
    // ═════════════════════════════════════════
    // 成员变量
    // ═════════════════════════════════════════
    
    /// 机械臂MoveIt规划接口
    std::shared_ptr<MoveGroup> m_moveGroup;
    
    /// 夹爪MoveIt规划接口
    std::shared_ptr<MoveGroup> m_gripperMoveGroup;
    
    /// 规划场景接口（用于管理碰撞物体）
    std::shared_ptr<PlanningSceneInterface> m_planningSceneInterface;
    
    /// PlanningScene 发布器（用于修改 ACM）
    PlanningScenePublisher m_planningScenePub;
    
    /// 日志记录器
    rclcpp::Logger m_logger;
};

#endif  // ROS2_LEARNING_ARM_BASICS_ARM_POSITION_CONTROLLER_HPP_
