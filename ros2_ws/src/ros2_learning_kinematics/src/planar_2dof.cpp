#include "ros2_learning_kinematics/planar_2dof.hpp"
#include <cmath>
#include <iostream>

namespace ros2_learning_kinematics
{

Planar2DOF::Planar2DOF(double l1, double l2)
    : l1_(l1), l2_(l2)
{
}

Planar2DOF::EndEffectorState Planar2DOF::forward(const JointState& joints)
{
    EndEffectorState ee;
    // FK 公式:
    // x = l1 * cos(t1) + l2 * cos(t1 + t2)
    // y = l1 * sin(t1) + l2 * sin(t1 + t2)
    
    ee.x = l1_ * std::cos(joints.theta1) + l2_ * std::cos(joints.theta1 + joints.theta2);
    ee.y = l1_ * std::sin(joints.theta1) + l2_ * std::sin(joints.theta1 + joints.theta2);
    
    return ee;
}

std::vector<Planar2DOF::JointState> Planar2DOF::inverse(const EndEffectorState& target)
{
    std::vector<JointState> solutions;
    
    double x = target.x;
    double y = target.y;
    
    // 1. 检查是否在工作空间内
    // dist^2 = x^2 + y^2
    double dist_sq = x*x + y*y;
    double max_reach = l1_ + l2_;
    double min_reach = std::abs(l1_ - l2_);
    
    if (dist_sq > max_reach * max_reach || dist_sq < min_reach * min_reach) {
        // 目标不可达
        return solutions;
    }
    
    // 2. 使用几何法求解
    // 余弦定理求 theta2
    // r^2 = l1^2 + l2^2 - 2*l1*l2*cos(180 - theta2)
    // r^2 = l1^2 + l2^2 + 2*l1*l2*cos(theta2)
    // cos(theta2) = (r^2 - l1^2 - l2^2) / (2*l1*l2)
    
    double cos_theta2 = (dist_sq - l1_*l1_ - l2_*l2_) / (2 * l1_ * l2_);
    
    // 限制浮点误差范围 [-1, 1]
    cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));
    
    // 求解 theta2 (两个解：正负)
    double theta2_up = std::acos(cos_theta2);   // Elbow Up (或 Down，取决于定义)
    double theta2_down = -theta2_up;            // 另一侧解
    
    // 求解 theta1
    // alpha = atan2(y, x)
    // beta = atan2(l2 * sin(theta2), l1 + l2 * cos(theta2))
    // theta1 = alpha - beta
    
    double alpha = std::atan2(y, x);
    
    // 解 1
    double beta_up = std::atan2(l2_ * std::sin(theta2_up), l1_ + l2_ * std::cos(theta2_up));
    double theta1_up = alpha - beta_up;
    solutions.push_back({theta1_up, theta2_up});
    
    // 解 2 (如果 theta2 不为 0，才会有第二个不同的解)
    if (std::abs(theta2_up) > 1e-6) {
        double beta_down = std::atan2(l2_ * std::sin(theta2_down), l1_ + l2_ * std::cos(theta2_down));
        double theta1_down = alpha - beta_down;
        solutions.push_back({theta1_down, theta2_down});
    }
    
    return solutions;
}

} // namespace ros2_learning_kinematics
