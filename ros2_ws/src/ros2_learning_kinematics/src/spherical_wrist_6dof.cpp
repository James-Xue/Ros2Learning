#include "ros2_learning_kinematics/spherical_wrist_6dof.hpp"
#include <iostream>

namespace ros2_learning_kinematics
{

SphericalWrist6DOF::SphericalWrist6DOF(double l1, double l2, double l3, double d4, double d6)
    : l1_(l1), l2_(l2), l3_(l3), d4_(d4), d6_(d6)
{
}

SphericalWrist6DOF::EndEffectorState SphericalWrist6DOF::forward(const JointState& joints)
{
    EndEffectorState ee;
    
    // 这里为了演示，我们使用标准的 D-H 矩阵连乘原理来计算 FK
    // 为了不引入过多的代码，我们直接使用 Eigen 库构造并相乘
    // 真实工业应用中，这里全都是写死的解析公式以换取极致的计算速度
    // 这里为了演示 D-H 参数法的数学魅力，我们将 6 个坐标系的齐次变换矩阵(4x4)相乘

    double t1 = joints.theta[0];
    double t2 = joints.theta[1];
    double t3 = joints.theta[2];
    double t4 = joints.theta[3];
    double t5 = joints.theta[4];
    double t6 = joints.theta[5];

    // =========================================================
    // 正运动学核心思想：【Denavit-Hartenberg (D-H) 连乘法】
    // =========================================================
    // 每一个变换矩阵 T_{i}^{i-1} 表示第 i 个关节相对于第 i-1 个关节的位姿变换。
    // 为了简化演示，我们假设这 6 个关节的 D-H 参数如下构型 (类似简单的串联机械臂)：
    // (注意：实际工业机器人的 D-H 表格可能有不同的平移/旋转方向定义，这里给出一种通用的模型)

    // 1. 基座旋转 (Z轴旋转 t1，沿 Z 轴平移 l1_ 到达肩关节)
    Eigen::Matrix4d T1_0 = Eigen::Matrix4d::Identity();
    T1_0(0,0) = cos(t1); T1_0(0,1) = -sin(t1);
    T1_0(1,0) = sin(t1); T1_0(1,1) = cos(t1);
    T1_0(2,3) = l1_;

    // 2. 肩部俯仰 (X轴旋转，沿 Z 轴平移 l2_ 到达肘关节)
    Eigen::Matrix4d T2_1 = Eigen::Matrix4d::Identity();
    T2_1(0,0) = cos(t2); T2_1(0,1) = -sin(t2);
    T2_1(1,0) = sin(t2); T2_1(1,1) = cos(t2);
    T2_1(2,3) = l2_;
    // 假设关节 2 的旋转轴是 Y 轴（相对前一坐标系）
    // 为了演示，此处直接写成标准形式的变换，具体轴向取决于具体的 D-H 建模

    // 3. 肘部俯仰 (同样 X/Y 平面旋转，沿 Z 轴平移 l3_ 到达腕中心)
    // 根据上面的 IK 逻辑，前三根杆件是在一个平面上的（如果 t1 固定）
    Eigen::Matrix4d T3_2 = Eigen::Matrix4d::Identity();
    T3_2(0,0) = cos(t3); T3_2(0,1) = -sin(t3);
    T3_2(1,0) = sin(t3); T3_2(1,1) = cos(t3);
    T3_2(2,3) = l3_;

    // 4. 手腕自转 (Wrist Roll)
    // 旋转 t4，Z 轴向前推进 d4_
    Eigen::Matrix4d T4_3 = Eigen::Matrix4d::Identity();
    T4_3(0,0) = cos(t4); T4_3(0,1) = -sin(t4);
    T4_3(1,0) = sin(t4); T4_3(1,1) = cos(t4);
    T4_3(2,3) = d4_;

    // 5. 手腕俯仰 (Wrist Pitch)
    // 旋转 t5，这里是相交点，没有平移
    Eigen::Matrix4d T5_4 = Eigen::Matrix4d::Identity();
    T5_4(0,0) = cos(t5); T5_4(0,1) = -sin(t5);
    T5_4(1,0) = sin(t5); T5_4(1,1) = cos(t5);
    
    // 6. 工具自转 (Tool Roll)
    // 旋转 t6，Z 轴向前伸出 d6_ 到达真正的机械爪末端
    Eigen::Matrix4d T6_5 = Eigen::Matrix4d::Identity();
    T6_5(0,0) = cos(t6); T6_5(0,1) = -sin(t6);
    T6_5(1,0) = sin(t6); T6_5(1,1) = cos(t6);
    T6_5(2,3) = d6_;

    // 最终的位姿变换矩阵 = T1 * T2 * T3 * T4 * T5 * T6
    Eigen::Matrix4d T_end = T1_0 * T2_1 * T3_2 * T4_3 * T5_4 * T6_5;

    // 提取位置 (右上角的 3x1 向量)
    ee.x = T_end(0, 3);
    ee.y = T_end(1, 3);
    ee.z = T_end(2, 3);

    // 提取姿态 (左上角的 3x3 旋转矩阵)
    ee.R = T_end.block<3, 3>(0, 0);

    // 计算欧拉角 (Roll, Pitch, Yaw) - 假设为 ZYX 顺序
    ee.yaw = std::atan2(ee.R(1, 0), ee.R(0, 0));
    ee.pitch = std::atan2(-ee.R(2, 0), std::sqrt(ee.R(2, 1)*ee.R(2, 1) + ee.R(2, 2)*ee.R(2, 2)));
    ee.roll = std::atan2(ee.R(2, 1), ee.R(2, 2));

    return ee;
}

std::vector<SphericalWrist6DOF::JointState> SphericalWrist6DOF::inverse(const EndEffectorState& target)
{
    std::vector<JointState> solutions;
    JointState sol;
    
    // =========================================================
    // 6-DOF 球形手腕逆运动学核心思想：【运动学解耦 (Kinematic Decoupling)】
    // =========================================================

    // 1. 获取目标位置 (p) 和 目标旋转矩阵 (R)
    Eigen::Vector3d p(target.x, target.y, target.z);
    Eigen::Matrix3d R = target.R;

    // 2. 第一步：计算"手腕中心 (Wrist Center, WC)"的位置
    // 因为最后三个关节(4,5,6)的旋转轴交于手腕中心，所以这三个关节的转动不会改变手腕中心的位置！
    // 也就是说，手腕中心的位置 *完全且仅仅* 由前三个关节(1,2,3)决定！
    // 我们可以沿着末端法向量 (Z轴)，往回退一个 d6 的长度，就找到了手腕中心。
    
    Eigen::Vector3d z_axis_of_end_effector = R.col(2); // 旋转矩阵的第三列就是末端Z轴在基座下的方向
    Eigen::Vector3d p_wc = p - d6_ * z_axis_of_end_effector; // 回退！

    double xc = p_wc.x();
    double yc = p_wc.y();
    double zc = p_wc.z();

    // 3. 第二步：求解前三个关节 (位置逆解)
    // 现在问题变成了一个只追求到达 (xc, yc, zc) 的 3-DOF 机械臂问题。
    
    // 求解 theta1 (基座旋转，看 XY 平面投影)
    sol.theta[0] = std::atan2(yc, xc); 
    
    // 求解 theta2 和 theta3 (这不就是我们刚才画在 2-DOF 里的几何余弦定理吗！)
    // 把 (xc, yc, zc) 投影到 theta1 确定的平面上，用余弦定理求大臂小臂夹角
    double r = std::sqrt(xc*xc + yc*yc);
    double s = zc - l1_; // 减去基座高度
    double D = (r*r + s*s - l2_*l2_ - l3_*l3_) / (2 * l2_ * l3_);
    
    // 假设求 elbow down 解
    sol.theta[2] = std::atan2(-std::sqrt(1 - D*D), D); // theta3
    sol.theta[1] = std::atan2(s, r) - std::atan2(l3_*std::sin(sol.theta[2]), l2_ + l3_*std::cos(sol.theta[2])); // theta2

    // 4. 第三步：求解后三个关节 (姿态逆解)
    // 我们已经算出了前三个关节，那么前三个关节带来的旋转是可以算出来的 (R_3_0)
    // 既然 目标总旋转 = 前三关节旋转 * 后三关节旋转 ( R = R_3_0 * R_6_3 )
    // 我们把前三个旋转矩阵逆乘过去，就得到了纯手腕需要转动的矩阵！
    // R_6_3 = [R_3_0]^T * R
    // 然后根据 R_6_3 这个 3x3 矩阵，用经典的欧拉角反三角函数，直接就能读出 theta4, theta5, theta6。

    // (伪代码略，直接赋值演示)
    sol.theta[3] = 0.0;
    sol.theta[4] = 0.0;
    sol.theta[5] = 0.0;

    solutions.push_back(sol);
    return solutions;
}

} // namespace ros2_learning_kinematics
