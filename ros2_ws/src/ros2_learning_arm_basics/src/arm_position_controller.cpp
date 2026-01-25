// arm_position_controller.cpp
// 机械臂位置控制器类的实现文件
#include "ros2_learning_arm_basics/arm_position_controller.hpp"

#include <chrono>
#include <string>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// ═══════════════════════════════════════════════════════════
// 常量定义 - 将所有魔法数字提取为命名常量
// ═══════════════════════════════════════════════════════════
namespace constants {
    // 夹爪参数
    constexpr double kGripperMaxWidth = 0.08;        // 夹爪最大宽度 8cm
    constexpr double kGripperOpenWidth = 0.035;      // 夹爪打开宽度 3.5cm
    constexpr double kGripperGraspWidth = 0.03;      // 抓取时夹爪宽度 3cm
    constexpr double kHandToFingertipOffset = 0.10;  // panda_hand原点到指尖的偏移量
    
    // 物体参数
    constexpr double kTargetBoxSize = 0.05;          // 目标立方体边长 5cm
    constexpr double kTableWidth = 0.6;              // 桌面宽度 60cm
    constexpr double kTableDepth = 0.8;              // 桌面深度 80cm
    constexpr double kTableThickness = 0.02;         // 桌面厚度 2cm
    
    // 位置参数
    constexpr double kObjectDistance = 0.4;          // 物体前方距离 40cm
    constexpr double kPrepareHeight = 0.35;          // 准备位置高度 35cm
    constexpr double kGraspHeight = 0.13;            // 抓取高度（物体中心 + hand偏移）
    constexpr double kLiftHeight = 0.5;              // 提升高度 50cm
    constexpr double kPlaceHeight = 0.3;             // 放置高度 30cm
    constexpr double kPlaceOffsetY = -0.3;           // 放置位置Y偏移 -30cm
    
    // 抓取方向参数（四元数：夹爪朝下）
    constexpr double kGraspOrientationX = 1.0;
    constexpr double kGraspOrientationY = 0.0;
    constexpr double kGraspOrientationZ = 0.0;
    constexpr double kGraspOrientationW = 0.0;
    
    // 时间参数（毫秒）
    constexpr int kShortDelay = 300;                 // 短延迟
    constexpr int kMediumDelay = 500;                // 中等延迟
    constexpr int kLongDelay = 1000;                 // 长延迟
    constexpr int kSceneSetupDelay = 2000;           // 场景设置延迟 2s
    
    // 链接和物体名称
    const std::string kGripperFrame = "panda_hand";
    const std::string kBaseFrame = "panda_link0";
    const std::string kTargetBoxId = "target_box";
    const std::string kTableId = "table";
    
    // 夹爪相关链接
    const std::vector<std::string> kGripperLinks = {
        "panda_hand",
        "panda_leftfinger", 
        "panda_rightfinger"
    };
}  // namespace constants

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
    
    // 创建规划场景接口
    m_planningSceneInterface = 
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    // 创建 PlanningScene 发布器（用于修改 ACM）
    m_planningScenePub = this->create_publisher<moveit_msgs::msg::PlanningScene>(
        "planning_scene", 10);
    
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
        m_moveGroup = std::make_shared<MoveGroup>(
            shared_from_this(), planning_group);
        
        // 关键修复：设置末端执行器链接为 panda_hand
        // panda_arm 默认使用 panda_link8（手腕），但我们需要控制夹爪位置
        m_moveGroup->setEndEffectorLink("panda_hand");
        
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
    auto success = (m_moveGroup->move() == ErrorCode::SUCCESS);
    
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
bool ArmPositionController::moveToPose(const Pose& target_pose) {
    RCLCPP_INFO(m_logger, "正在移动到笛卡尔空间目标位置");
    RCLCPP_INFO(m_logger, "  位置: [%.3f, %.3f, %.3f]", 
                target_pose.position.x, 
                target_pose.position.y, 
                target_pose.position.z);
    
    // 设置目标位姿
    m_moveGroup->setPoseTarget(target_pose);
    
    // 规划运动路径
    Plan plan;
    bool success = (m_moveGroup->plan(plan) == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "规划成功，正在执行...");
        m_moveGroup->execute(plan);
    } else {
        RCLCPP_WARN(m_logger, "规划失败");
    }
    return success;
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
    auto success = (m_moveGroup->move() == ErrorCode::SUCCESS);
    
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
    Pose target_pose;
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

/**
 * @brief 画正方形演示
 * 
 * 使用笛卡尔路径规划让末端沿正方形路径运动
 * 关键API：computeCartesianPath() - 强制沿直线运动
 */
void ArmPositionController::drawSquare() {
    RCLCPP_INFO(m_logger, "\n========================================");
    RCLCPP_INFO(m_logger, "  笛卡尔路径演示：画正方形");
    RCLCPP_INFO(m_logger, "========================================\n");
    
    // ═══════════════════════════════════════
    // 步骤1: 移动到起始位置
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "[1] 移动到起始位置");
    
    Pose start_pose;
    start_pose.orientation.w = 1.0;  // 保持水平朝向
    start_pose.position.x = 0.4;     // 前方40cm
    start_pose.position.y = 0.1;     // 左侧10cm（正方形左下角）
    start_pose.position.z = 0.4;     // 高度40cm
    
    m_moveGroup->setPoseTarget(start_pose);
    Plan plan;
    
    if (m_moveGroup->plan(plan) == ErrorCode::SUCCESS) {
        RCLCPP_INFO(m_logger, "  到达起始点: (%.2f, %.2f, %.2f)", 
                    start_pose.position.x, start_pose.position.y, start_pose.position.z);
        m_moveGroup->execute(plan);
    } else {
        RCLCPP_ERROR(m_logger, "  移动到起始位置失败！");
        return;
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 步骤2: 定义正方形的4个顶点
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[2] 规划正方形路径");
    
    double square_size = 0.1;  // 正方形边长10cm
    
    std::vector<Pose> waypoints;
    
    // 当前位置作为起点（左下角）
    waypoints.push_back(start_pose);
    
    // 顶点1: 右下角（y方向-）
    Pose corner1 = start_pose;
    corner1.position.y -= square_size;
    waypoints.push_back(corner1);
    RCLCPP_INFO(m_logger, "  顶点1（右下）: (%.2f, %.2f, %.2f)", 
                corner1.position.x, corner1.position.y, corner1.position.z);
    
    // 顶点2: 右上角（x方向+）
    Pose corner2 = corner1;
    corner2.position.x += square_size;
    waypoints.push_back(corner2);
    RCLCPP_INFO(m_logger, "  顶点2（右上）: (%.2f, %.2f, %.2f)", 
                corner2.position.x, corner2.position.y, corner2.position.z);
    
    // 顶点3: 左上角（y方向+）
    Pose corner3 = corner2;
    corner3.position.y += square_size;
    waypoints.push_back(corner3);
    RCLCPP_INFO(m_logger, "  顶点3（左上）: (%.2f, %.2f, %.2f)", 
                corner3.position.x, corner3.position.y, corner3.position.z);
    
    // 顶点4: 回到起点（闭合正方形）
    waypoints.push_back(start_pose);
    RCLCPP_INFO(m_logger, "  顶点4（回到起点）");
    
    // ═══════════════════════════════════════
    // 步骤3: 计算笛卡尔路径
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[3] 计算笛卡尔路径（强制直线运动）");
    
    Trajectory trajectory;
    const double eef_step = 0.01;  // 末端步长1cm（路径分辨率）
    
    double fraction = m_moveGroup->computeCartesianPath(
        waypoints,      // 路径点
        eef_step,       // 步长
        trajectory      // 输出轨迹
    );
    
    RCLCPP_INFO(m_logger, "  路径规划完成度: %.1f%%", fraction * 100.0);
    RCLCPP_INFO(m_logger, "  轨迹点数: %zu", trajectory.joint_trajectory.points.size());
    
    if (fraction < 0.99) {  // 如果没有完成100%
        RCLCPP_WARN(m_logger, "  警告：路径未完全规划！可能遇到奇异点或障碍");
    }
    
    // ═══════════════════════════════════════
    // 步骤4: 执行轨迹
    // ═══════════════════════════════════════
    if (fraction > 0.5) {  // 至少完成50%才执行
        RCLCPP_INFO(m_logger, "\n[4] 开始画正方形...");
        
        // 直接执行笛卡尔路径
        // computeCartesianPath 已经生成了完整的轨迹，包含时间信息
        Plan square_plan;
        square_plan.trajectory = trajectory;
        
        m_moveGroup->execute(square_plan);
        
        RCLCPP_INFO(m_logger, "\n✓ 正方形绘制完成！");
    } else {
        RCLCPP_ERROR(m_logger, "  路径规划失败率太高，取消执行");
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 步骤5: 返回ready姿态
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[5] 返回ready姿态");
    moveToNamedTarget("ready");
    
    RCLCPP_INFO(m_logger, "\n========================================");
    RCLCPP_INFO(m_logger, "  正方形演示完成！");
    RCLCPP_INFO(m_logger, "========================================\n");
}

// ═══════════════════════════════════════════════════════════
// 夹爪控制方法实现
// ═══════════════════════════════════════════════════════════

/**
 * @brief 初始化夹爪控制接口
 * 
 * 创建"hand"规划组的MoveGroup接口
 */
bool ArmPositionController::initializeGripper() {
    try {
        // 创建夹爪MoveGroup接口
        m_gripperMoveGroup = std::make_shared<MoveGroup>(
            shared_from_this(), "hand");
        
        RCLCPP_INFO(m_logger, "\n========================================");
        RCLCPP_INFO(m_logger, "  夹爪控制接口初始化");
        RCLCPP_INFO(m_logger, "========================================");
        RCLCPP_INFO(m_logger, "规划组: %s", m_gripperMoveGroup->getPlanningFrame().c_str());
        RCLCPP_INFO(m_logger, "末端执行器: hand");
        RCLCPP_INFO(m_logger, "可用的预定义姿态: open, close");
        RCLCPP_INFO(m_logger, "========================================\n");
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(m_logger, "夹爪初始化失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 打开夹爪
 * 
 * 移动到预定义的"open"姿态（3.5cm宽度）
 */
bool ArmPositionController::openGripper() {
    RCLCPP_INFO(m_logger, "正在打开夹爪...");
    
    // 设置目标为预定义的"open"姿态
    m_gripperMoveGroup->setNamedTarget("open");
    
    // 执行运动
    bool success = (m_gripperMoveGroup->move() == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "✓ 夹爪已打开（宽度: %.1fcm）", 
                    constants::kGripperOpenWidth * 100.0);
    } else {
        RCLCPP_WARN(m_logger, "✗ 打开夹爪失败");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kMediumDelay));
    return success;
}

/**
 * @brief 闭合夹爪
 * 
 * 移动到预定义的"close"姿态（完全闭合）
 */
bool ArmPositionController::closeGripper() {
    RCLCPP_INFO(m_logger, "正在闭合夹爪...");
    
    // 设置目标为预定义的"close"姿态
    m_gripperMoveGroup->setNamedTarget("close");
    
    // 执行运动
    bool success = (m_gripperMoveGroup->move() == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "✓ 夹爪已闭合");
    } else {
        RCLCPP_WARN(m_logger, "✗ 闭合夹爪失败");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kMediumDelay));
    return success;
}

/**
 * @brief 设置夹爪宽度
 * 
 * @param width 夹爪宽度（单位：米，范围：0.0 ~ 0.08）
 */
bool ArmPositionController::setGripperWidth(double width) {
    // 限制范围，使用常量
    if (width < 0.0) width = 0.0;
    if (width > constants::kGripperMaxWidth) width = constants::kGripperMaxWidth;
    
    RCLCPP_INFO(m_logger, "正在设置夹爪宽度: %.3f m (%.1f cm)", 
                width, width * 100.0);
    
    // Panda夹爪有两个关节，每个关节控制一个手指
    // 每个手指移动 width/2 的距离
    std::vector<double> joint_values = {width / 2.0, width / 2.0};
    
    m_gripperMoveGroup->setJointValueTarget(joint_values);
    
    bool success = (m_gripperMoveGroup->move() == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "✓ 夹爪宽度已设置");
    } else {
        RCLCPP_WARN(m_logger, "✗ 设置夹爪宽度失败");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kMediumDelay));
    return success;
}

// ═══════════════════════════════════════════════════════════
// 物体管理和真实抓取方法实现
// ═══════════════════════════════════════════════════════════

/**
 * @brief 在场景中生成目标物体
 * 
 * 在机械臂前方生成一个5cm×5cm×5cm的立方体
 */
void ArmPositionController::spawnTargetObject() {
    RCLCPP_INFO(m_logger, "\n🎁 正在在场景中生成目标物体...");
    
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // ========================================
    // 1. 创建桌面碰撞物体
    // ========================================
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = constants::kBaseFrame;
    table.id = constants::kTableId;
    
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = constants::kTableWidth;   // x: 60cm
    table_primitive.dimensions[1] = constants::kTableDepth;    // y: 80cm  
    table_primitive.dimensions[2] = constants::kTableThickness; // z: 2cm (桌面厚度)
    
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = constants::kObjectDistance;
    table_pose.position.y = 0.0;
    table_pose.position.z = -constants::kTableThickness / 2.0;  // 桌面中心，顶面在 z=0
    table_pose.orientation.w = 1.0;
    
    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);
    
    // ========================================
    // 2. 创建目标物体（放在桌面上）
    // ========================================
    moveit_msgs::msg::CollisionObject target_box;
    target_box.header.frame_id = constants::kBaseFrame;
    target_box.id = constants::kTargetBoxId;
    
    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = constants::kTargetBoxSize;  // x: 5cm
    box_primitive.dimensions[1] = constants::kTargetBoxSize;  // y: 5cm
    box_primitive.dimensions[2] = constants::kTargetBoxSize;  // z: 5cm
    
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = constants::kObjectDistance;   // 前方40cm
    box_pose.position.y = 0.0;   // 中央
    box_pose.position.z = constants::kTargetBoxSize / 2.0; // 桌面上方（立方体一半高度）
    box_pose.orientation.w = 1.0;
    
    target_box.primitives.push_back(box_primitive);
    target_box.primitive_poses.push_back(box_pose);
    target_box.operation = target_box.ADD;
    collision_objects.push_back(target_box);
    
    // 添加所有物体到场景
    m_planningSceneInterface->applyCollisionObjects(collision_objects);
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kMediumDelay));
    
    RCLCPP_INFO(m_logger, "✓ 场景物体已生成");
    RCLCPP_INFO(m_logger, "  - 桌面: 60cm × 80cm × 2cm (顶面在 z=0)");
    RCLCPP_INFO(m_logger, "  - 物体: 5cm × 5cm × 5cm");
    RCLCPP_INFO(m_logger, "  - 位置: (%.2f, %.2f, %.2f)\n", 
                box_pose.position.x, box_pose.position.y, box_pose.position.z);
}

/**
 * @brief 从场景中移除目标物体
 */
void ArmPositionController::removeTargetObject() {
    RCLCPP_INFO(m_logger, "正在移除场景物体...");
    
    std::vector<std::string> object_ids;
    object_ids.push_back(constants::kTargetBoxId);
    object_ids.push_back(constants::kTableId);  // 也移除桌面
    
    m_planningSceneInterface->removeCollisionObjects(object_ids);
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kShortDelay));
    
    RCLCPP_INFO(m_logger, "✓ 场景物体已移除\n");
}

/**
 * @brief 将物体附加到夹爪
 * 
 * 实现物理附加，物体将跟随夹爪移动
 */
void ArmPositionController::attachObjectToGripper(const std::string& object_id) {
    RCLCPP_INFO(m_logger, "🔗 正在将物体 '%s' 附加到夹爪...", object_id.c_str());
    
    // 创建附加碰撞物体消息
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "panda_hand";  // 附加到夹爪链接
    attached_object.object.id = object_id;
    attached_object.object.operation = attached_object.object.ADD;
    
    // 指定允许接触的链接（避免碰撞检测误报）
    attached_object.touch_links = std::vector<std::string>{
        "panda_hand", 
        "panda_leftfinger", 
        "panda_rightfinger"
    };
    
    // 应用附加
    m_planningSceneInterface->applyAttachedCollisionObject(attached_object);
    
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    
    RCLCPP_INFO(m_logger, "✓ 物体已附加到夹爪，将跟随夹爪移动\n");
}

/**
 * @brief 从夹爪分离物体
 * 
 * 物体将留在当前位置
 */
void ArmPositionController::detachObjectFromGripper(const std::string& object_id) {
    RCLCPP_INFO(m_logger, "🔓 正在从夹爪分离物体 '%s'...", object_id.c_str());
    
    // 创建分离消息
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = object_id;
    detach_object.object.operation = detach_object.object.REMOVE;
    
    // 应用分离
    m_planningSceneInterface->applyAttachedCollisionObject(detach_object);
    
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    
    RCLCPP_INFO(m_logger, "✓ 物体已从夹爪分离，留在当前位置\n");
}

/**
 * @brief 允许/禁止夹爪与指定物体碰撞
 * 
 * 通过发布 PlanningScene 消息修改 ACM
 * 
 * @param object_id 物体ID
 * @param allow true=允许碰撞, false=禁止碰撞
 */
void ArmPositionController::allowObjectCollision(const std::string& object_id, bool allow) {
    RCLCPP_INFO(m_logger, "🔧 正在%s夹爪与物体 '%s' 的碰撞检测...", 
                allow ? "禁用" : "启用", object_id.c_str());
    
    // 创建 PlanningScene 消息
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;
    
    // 使用 default_entry_names/values 方式修改 ACM
    // 这种方式更简单且不会破坏现有的碰撞矩阵
    auto& acm = planning_scene_msg.allowed_collision_matrix;
    
    // 夹爪相关链接
    std::vector<std::string> gripper_links = {
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger"
    };
    
    // 设置允许物体与所有夹爪链接碰撞
    acm.default_entry_names.push_back(object_id);
    acm.default_entry_values.push_back(allow);
    
    // 为每个夹爪链接单独设置
    for (const auto& link : gripper_links) {
        acm.default_entry_names.push_back(link);
        acm.default_entry_values.push_back(allow);
    }
    
    // 发布更新
    m_planningScenePub->publish(planning_scene_msg);
    
    // 等待更新生效
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(m_logger, "✓ 碰撞检测已%s\n", allow ? "禁用" : "启用");
}

// ═══════════════════════════════════════════════════════════
// Pick-and-Place 辅助函数实现
// ═══════════════════════════════════════════════════════════

/**
 * @brief 移动到物体上方的准备位置并打开夹爪
 * 
 * @param object_id 目标物体ID（用于日志）
 * @return true 成功
 * @return false 失败
 */
bool ArmPositionController::moveToPreGraspPosition(const std::string& object_id) {
    RCLCPP_INFO(m_logger, "[1/4] 移动到物体 '%s' 上方准备位置", object_id.c_str());
    
    // 构造准备位置姿态（夹爪朝下）
    Pose pre_grasp_pose;
    pre_grasp_pose.orientation.x = constants::kGraspOrientationX;
    pre_grasp_pose.orientation.y = constants::kGraspOrientationY;
    pre_grasp_pose.orientation.z = constants::kGraspOrientationZ;
    pre_grasp_pose.orientation.w = constants::kGraspOrientationW;
    pre_grasp_pose.position.x = constants::kObjectDistance;
    pre_grasp_pose.position.y = 0.0;
    pre_grasp_pose.position.z = constants::kPrepareHeight;
    
    if (!moveToPose(pre_grasp_pose)) {
        RCLCPP_ERROR(m_logger, "✗ 移动到准备位置失败");
        return false;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kLongDelay));
    
    // 打开夹爪
    RCLCPP_INFO(m_logger, "打开夹爪准备抓取");
    openGripper();
    
    return true;
}

/**
 * @brief 抓取指定物体
 * 
 * 严格按照原始顺序：禁用碰撞 -> 下降 -> 闭合夹爪 -> 附加物体 -> 提升
 * 
 * @param object_id 目标物体ID
 * @param above_pose 物体上方的姿态（用于构造抓取姿态）
 * @return true 成功抓取
 * @return false 抓取失败
 */
bool ArmPositionController::graspObject(const std::string& object_id) {
    RCLCPP_INFO(m_logger, "\n[2/4] 执行物体抓取");
    
    // 关键：先允许夹爪与物体碰撞
    allowObjectCollision(object_id, true);
    
    // 下降到抓取位置
    RCLCPP_INFO(m_logger, "下降到抓取位置");
    Pose grasp_pose;
    grasp_pose.orientation.x = constants::kGraspOrientationX;
    grasp_pose.orientation.y = constants::kGraspOrientationY;
    grasp_pose.orientation.z = constants::kGraspOrientationZ;
    grasp_pose.orientation.w = constants::kGraspOrientationW;
    grasp_pose.position.x = constants::kObjectDistance;
    grasp_pose.position.y = 0.0;
    grasp_pose.position.z = constants::kGraspHeight;
    
    if (!moveToPose(grasp_pose)) {
        RCLCPP_ERROR(m_logger, "✗ 下降到抓取位置失败");
        return false;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kLongDelay));
    
    // 关键顺序：先闭合夹爪，再附加物体（与原始代码保持一致）
    RCLCPP_INFO(m_logger, "闭合夹爪抓取物体");
    setGripperWidth(constants::kGripperGraspWidth);
    
    // 附加物体到夹爪
    attachObjectToGripper(object_id);
    RCLCPP_INFO(m_logger, "✓ 物体已被抓取！");
    
    // 提升物体
    RCLCPP_INFO(m_logger, "提升物体");
    Pose lift_pose = grasp_pose;
    lift_pose.position.z = constants::kLiftHeight;
    
    if (!moveToPose(lift_pose)) {
        RCLCPP_ERROR(m_logger, "✗ 提升物体失败");
        return false;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kLongDelay));
    
    return true;
}

/**
 * @brief 将物体放置到目标位置
 * 
 * @param object_id 目标物体ID
 * @return true 成功放置
 * @return false 放置失败
 */
bool ArmPositionController::placeObject(const std::string& object_id) {
    RCLCPP_INFO(m_logger, "\n[3/4] 移动到放置位置");
    
    // 移动到放置位置
    Pose place_pose;
    place_pose.orientation.w = 1.0;
    place_pose.position.x = constants::kObjectDistance;
    place_pose.position.y = constants::kPlaceOffsetY;
    place_pose.position.z = constants::kPlaceHeight;
    
    if (!moveToPose(place_pose)) {
        RCLCPP_ERROR(m_logger, "✗ 移动到放置位置失败");
        return false;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kLongDelay));
    
    // 分离物体并打开夹爪
    RCLCPP_INFO(m_logger, "分离物体并打开夹爪");
    detachObjectFromGripper(object_id);
    openGripper();
    
    RCLCPP_INFO(m_logger, "✓ 物体已放置！");
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kLongDelay));
    
    return true;
}

/**
 * @brief 清理场景并返回初始位置
 * 
 * @param object_id 要移除的物体ID
 */
void ArmPositionController::cleanupAndReturnHome(const std::string& object_id) {
    RCLCPP_INFO(m_logger, "\n[4/4] 返回初始位置并清理");
    
    // 恢复碰撞检测（重要：在移动前恢复正常碰撞设置）
    allowObjectCollision(object_id, false);
    
    // 返回 ready 姿态
    moveToNamedTarget("ready");
    closeGripper();  // 闭合夹爪（ready姿态只控制手臂，需单独闭合夹爪）
    
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kLongDelay));
    
    // 清理：移除物体
    RCLCPP_INFO(m_logger, "移除场景中的物体");
    removeTargetObject();
}

/**
 * @brief 真实的抓取和放置演示
 * 
 * 包含物体生成、附加、分离的完整流程
 * 重构为模块化架构，但保持原始操作顺序
 */
void ArmPositionController::runRealisticPickAndPlace() {
    RCLCPP_INFO(m_logger, "\n╔════════════════════════════════════════════════╗");
    RCLCPP_INFO(m_logger, "║  🎯 真实物体抓取和放置演示                    ║");
    RCLCPP_INFO(m_logger, "╚════════════════════════════════════════════════╝\n");
    
    const std::string object_id = constants::kTargetBoxId;
    
    // ═══════════════════════════════════════
    // 场景设置
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "[场景设置] 生成目标物体");
    spawnTargetObject();
    rclcpp::sleep_for(std::chrono::milliseconds(constants::kSceneSetupDelay));
    
    // ═══════════════════════════════════════
    // 执行抓取放置流程
    // ═══════════════════════════════════════
    
    // 步骤1: 移动到准备位置
    if (!moveToPreGraspPosition(object_id)) {
        RCLCPP_ERROR(m_logger, "❌ 演示失败：无法移动到准备位置");
        cleanupAndReturnHome(object_id);
        return;
    }
    
    // 步骤2: 抓取物体
    if (!graspObject(object_id)) {
        RCLCPP_ERROR(m_logger, "❌ 演示失败：无法抓取物体");
        cleanupAndReturnHome(object_id);
        return;
    }
    
    // 步骤3: 放置物体
    if (!placeObject(object_id)) {
        RCLCPP_ERROR(m_logger, "❌ 演示失败：无法放置物体");
        cleanupAndReturnHome(object_id);
        return;
    }
    
    // ═══════════════════════════════════════
    // 清理和返回
    // ═══════════════════════════════════════
    cleanupAndReturnHome(object_id);
    
    RCLCPP_INFO(m_logger, "\n╔════════════════════════════════════════════════╗");
    RCLCPP_INFO(m_logger, "║  ✅ 真实物体抓取演示完成！                    ║");
    RCLCPP_INFO(m_logger, "╚════════════════════════════════════════════════╝\n");
}

