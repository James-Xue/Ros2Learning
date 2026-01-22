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
        m_moveGroup = std::make_shared<MoveGroup>(
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
void ArmPositionController::moveToPose(const Pose& target_pose) {
    RCLCPP_INFO(m_logger, "正在移动到笛卡尔空间目标位置");
    RCLCPP_INFO(m_logger, "  位置: [%.3f, %.3f, %.3f]", 
                target_pose.position.x, 
                target_pose.position.y, 
                target_pose.position.z);
    
    // 设置目标位姿
    m_moveGroup->setPoseTarget(target_pose);
    
    // 规划运动路径
    Plan plan;
    auto success = (m_moveGroup->plan(plan) == ErrorCode::SUCCESS);
    
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
void ArmPositionController::openGripper() {
    RCLCPP_INFO(m_logger, "正在打开夹爪...");
    
    // 设置目标为预定义的"open"姿态
    m_gripperMoveGroup->setNamedTarget("open");
    
    // 执行运动
    auto success = (m_gripperMoveGroup->move() == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "✓ 夹爪已打开（宽度: 3.5cm）");
    } else {
        RCLCPP_WARN(m_logger, "✗ 打开夹爪失败");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
}

/**
 * @brief 闭合夹爪
 * 
 * 移动到预定义的"close"姿态（完全闭合）
 */
void ArmPositionController::closeGripper() {
    RCLCPP_INFO(m_logger, "正在闭合夹爪...");
    
    // 设置目标为预定义的"close"姿态
    m_gripperMoveGroup->setNamedTarget("close");
    
    // 执行运动
    auto success = (m_gripperMoveGroup->move() == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "✓ 夹爪已闭合");
    } else {
        RCLCPP_WARN(m_logger, "✗ 闭合夹爪失败");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
}

/**
 * @brief 设置夹爪宽度
 * 
 * @param width 夹爪宽度（单位：米，范围：0.0 ~ 0.08）
 */
void ArmPositionController::setGripperWidth(double width) {
    // 限制范围
    if (width < 0.0) width = 0.0;
    if (width > 0.08) width = 0.08;
    
    RCLCPP_INFO(m_logger, "正在设置夹爪宽度: %.3f m (%.1f cm)", 
                width, width * 100.0);
    
    // Panda夹爪有两个关节，每个关节控制一个手指
    // 每个手指移动 width/2 的距离
    std::vector<double> joint_values = {width / 2.0, width / 2.0};
    
    m_gripperMoveGroup->setJointValueTarget(joint_values);
    
    auto success = (m_gripperMoveGroup->move() == ErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(m_logger, "✓ 夹爪宽度已设置");
    } else {
        RCLCPP_WARN(m_logger, "✗ 设置夹爪宽度失败");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
}

/**
 * @brief 抓取和放置演示
 * 
 * 完整的抓取序列演示
 */
void ArmPositionController::runPickAndPlaceDemo() {
    RCLCPP_INFO(m_logger, "\n╔════════════════════════════════════════╗");
    RCLCPP_INFO(m_logger, "║  🤖 抓取和放置演示                    ║");
    RCLCPP_INFO(m_logger, "╚════════════════════════════════════════╝\n");
    
    // ═══════════════════════════════════════
    // 步骤1: 移动到物体上方（准备位置）
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "[1/7] 移动到物体上方");
    
    Pose above_object;
    above_object.orientation.w = 1.0;
    above_object.position.x = 0.4;   // 前方40cm
    above_object.position.y = 0.0;   // 中央
    above_object.position.z = 0.4;   // 上方40cm
    
    moveToPose(above_object);
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 步骤2: 打开夹爪
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[2/7] 打开夹爪准备抓取");
    openGripper();
    
    // ═══════════════════════════════════════
    // 步骤3: 下降到抓取位置
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[3/7] 下降到抓取位置");
    
    Pose grasp_pose = above_object;
    grasp_pose.position.z = 0.2;  // 下降到20cm高度
    
    moveToPose(grasp_pose);
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 步骤4: 闭合夹爪（模拟抓取物体）
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[4/7] 闭合夹爪抓取物体");
    setGripperWidth(0.02);  // 设置为2cm（模拟抓取小物体）
    RCLCPP_INFO(m_logger, "✓ 物体已抓取！");
    
    // ═══════════════════════════════════════
    // 步骤5: 提升物体
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[5/7] 提升物体");
    
    Pose lift_pose = grasp_pose;
    lift_pose.position.z = 0.5;  // 提升到50cm高度
    
    moveToPose(lift_pose);
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 步骤6: 移动到放置位置
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[6/7] 移动到放置位置");
    
    Pose place_pose;
    place_pose.orientation.w = 1.0;
    place_pose.position.x = 0.4;
    place_pose.position.y = -0.3;  // 移动到右侧30cm
    place_pose.position.z = 0.3;   // 放置高度30cm
    
    moveToPose(place_pose);
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 步骤7: 打开夹爪释放物体
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[7/7] 打开夹爪释放物体");
    openGripper();
    RCLCPP_INFO(m_logger, "✓ 物体已放置！");
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // ═══════════════════════════════════════
    // 完成：返回ready姿态
    // ═══════════════════════════════════════
    RCLCPP_INFO(m_logger, "\n[完成] 返回ready姿态");
    moveToNamedTarget("ready");
    
    RCLCPP_INFO(m_logger, "\n╔════════════════════════════════════════╗");
    RCLCPP_INFO(m_logger, "║  ✅ 抓取和放置演示完成！              ║");
    RCLCPP_INFO(m_logger, "╚════════════════════════════════════════╝\n");
}

