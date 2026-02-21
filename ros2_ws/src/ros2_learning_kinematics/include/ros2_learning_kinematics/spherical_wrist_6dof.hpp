#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace ros2_learning_kinematics
{

/**
 * @brief 经典 6 自由度机械臂（带球形手腕）运动学求解器
 * 
 * 这是一个简化的 6-DOF 模型，假设：
 * 1. 关节 1, 2, 3 负责位置（类似 PUMA 560 或 UR 的前三轴）
 * 2. 关节 4, 5, 6 轴线交于一点，构成“球形手腕”（Spherical Wrist），负责姿态
 * 
 * 这种结构可以通过“运动学解耦”求得封闭解（解析解）。
 */
class SphericalWrist6DOF
{
public:
    struct JointState {
        double theta[6]; // 6个关节角（弧度）
    };

    struct EndEffectorState {
        // 末端位置 (x, y, z)
        double x, y, z;
        // 末端姿态 (Roll, Pitch, Yaw)
        double roll, pitch, yaw;
        // 等效的旋转矩阵 (3x3)
        Eigen::Matrix3d R;
    };

    /**
     * @brief 构造函数，需要传入机械臂的几何尺寸（连杆长度）
     * 为简化演示，这里假设一种简单的垂直串联构型
     */
    SphericalWrist6DOF(double l1, double l2, double l3, double d4, double d6);

    /**
     * @brief 正运动学 (Forward Kinematics)
     * 给定 6 个关节角，计算末端位姿 (位置 + 旋转矩阵/欧拉角)
     */
    EndEffectorState forward(const JointState& joints);

    /**
     * @brief 逆运动学 (Inverse Kinematics)
     * 给定末端期望位姿，计算可能的关节角组合。
     * 6轴机械臂通常有 8 组解（肩部左/右，肘部上/下，手腕翻转/不翻转）。
     * 此处为演示，我们将仅返回最基础的一组或两组解。
     */
    std::vector<JointState> inverse(const EndEffectorState& target);

private:
    // 机械臂几何参数
    double l1_; // 基座高度
    double l2_; // 大臂长度
    double l3_; // 小臂长度
    double d4_; // 腕部偏置
    double d6_; // 末端工具长度
};

} // namespace ros2_learning_kinematics
