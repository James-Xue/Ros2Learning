#pragma once

#include <vector>
#include <cmath>
#include <optional>

namespace ros2_learning_kinematics
{

/**
 * @brief 平面 2 自由度机械臂运动学求解器
 * 
 * 这是一个纯数学类，没有任何 ROS 依赖。
 * 
 * 机械臂参数:
 * - l1: 第一杆长度
 * - l2: 第二杆长度
 * 
 * 坐标系:
 * - Base 在 (0,0)
 * - 关节 1 在 Base
 * - 关节 2 在 l1 末端
 */
class Planar2DOF
{
public:
    struct JointState {
        double theta1; // 弧度
        double theta2; // 弧度
    };

    struct EndEffectorState {
        double x;
        double y;
    };

    Planar2DOF(double l1 = 1.0, double l2 = 1.0);

    /**
     * @brief 正运动学 (Forward Kinematics)
     * 给定关节角，求末端坐标。
     */
    EndEffectorState forward(const JointState& joints);

    /**
     * @brief 逆运动学 (Inverse Kinematics)
     * 给定末端坐标，求关节角。
     * 
     * 注意：可能有 0 个、1 个或 2 个解（Elbow Up / Elbow Down）。
     * 这里我们返回所有可能的解。
     */
    std::vector<JointState> inverse(const EndEffectorState& target);

private:
    double l1_;
    double l2_;
};

} // namespace ros2_learning_kinematics
