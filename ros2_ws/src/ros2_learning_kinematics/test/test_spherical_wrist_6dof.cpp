#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Dense>
#include "ros2_learning_kinematics/spherical_wrist_6dof.hpp"

using ros2_learning_kinematics::SphericalWrist6DOF;

// 注意：当前实现中所有关节旋转轴均为 Z 轴（简化演示模型），
// 因此末端位置始终沿全局 Z 轴堆叠，与关节角度无关。
// 这些测试验证代码的数学自洽性，而非真实 6-DOF 运动学。

// 测试用机械臂参数
constexpr double L1 = 0.3;   // 基座高度
constexpr double L2 = 0.5;   // 大臂
constexpr double L3 = 0.4;   // 小臂
constexpr double D4 = 0.1;   // 腕部偏置
constexpr double D6 = 0.08;  // 工具长度
constexpr double TOTAL_Z = L1 + L2 + L3 + D4 + D6;  // = 1.38

constexpr double EPS = 1e-9;

// 辅助函数：创建默认测试用机械臂
SphericalWrist6DOF make_arm()
{
    return SphericalWrist6DOF(L1, L2, L3, D4, D6);
}

// ─── 正运动学 (FK) 测试 ──────────────────────────────────────────────────────

// 零位：所有关节角为 0 → 末端正上方，z = 各段之和
TEST(SphericalWrist6DOFTest, FKAllZeroAngles)
{
    auto arm = make_arm();
    SphericalWrist6DOF::JointState joints{};    // 全零初始化

    auto ee = arm.forward(joints);

    EXPECT_NEAR(ee.x, 0.0, EPS);
    EXPECT_NEAR(ee.y, 0.0, EPS);
    EXPECT_NEAR(ee.z, TOTAL_Z, EPS);
}

// 零位时旋转矩阵应为单位矩阵
TEST(SphericalWrist6DOFTest, FKZeroAnglesRotationIsIdentity)
{
    auto arm = make_arm();
    SphericalWrist6DOF::JointState joints{};

    auto ee = arm.forward(joints);

    // 对角元素应为 1，非对角元素应为 0
    EXPECT_NEAR(ee.R(0, 0), 1.0, EPS);
    EXPECT_NEAR(ee.R(1, 1), 1.0, EPS);
    EXPECT_NEAR(ee.R(2, 2), 1.0, EPS);
    EXPECT_NEAR(ee.R(0, 1), 0.0, EPS);
    EXPECT_NEAR(ee.R(1, 0), 0.0, EPS);
}

// 零位时欧拉角 roll/pitch/yaw 均为 0
TEST(SphericalWrist6DOFTest, FKZeroAnglesEulerAllZero)
{
    auto arm = make_arm();
    SphericalWrist6DOF::JointState joints{};

    auto ee = arm.forward(joints);

    EXPECT_NEAR(ee.roll,  0.0, EPS);
    EXPECT_NEAR(ee.pitch, 0.0, EPS);
    EXPECT_NEAR(ee.yaw,   0.0, EPS);
}

// 由于所有关节均绕 Z 旋转，位置应与关节角无关（仅沿全局 Z 堆叠）
TEST(SphericalWrist6DOFTest, FKPositionInvariantToZRotations)
{
    auto arm = make_arm();

    SphericalWrist6DOF::JointState joints_a{};
    joints_a.theta[0] = M_PI / 4.0;
    joints_a.theta[2] = M_PI / 3.0;

    SphericalWrist6DOF::JointState joints_b{};
    joints_b.theta[1] = -M_PI / 6.0;
    joints_b.theta[3] = M_PI / 2.0;

    auto ee_a = arm.forward(joints_a);
    auto ee_b = arm.forward(joints_b);

    // 位置应始终在 (0, 0, TOTAL_Z)
    EXPECT_NEAR(ee_a.x, 0.0, EPS);
    EXPECT_NEAR(ee_a.y, 0.0, EPS);
    EXPECT_NEAR(ee_a.z, TOTAL_Z, EPS);

    EXPECT_NEAR(ee_b.x, 0.0, EPS);
    EXPECT_NEAR(ee_b.y, 0.0, EPS);
    EXPECT_NEAR(ee_b.z, TOTAL_Z, EPS);
}

// 所有关节角叠加为 Rz：最终 yaw = sum(theta[0..5])
TEST(SphericalWrist6DOFTest, FKYawEqualsAngleSum)
{
    auto arm = make_arm();
    SphericalWrist6DOF::JointState joints{};
    joints.theta[0] = M_PI / 6.0;   // 30°
    joints.theta[1] = M_PI / 6.0;   // 30°
    joints.theta[2] = M_PI / 6.0;   // 30°
    // 其余为 0 → 总计 90°

    auto ee = arm.forward(joints);

    EXPECT_NEAR(ee.yaw, M_PI / 2.0, 1e-9);
    EXPECT_NEAR(ee.pitch, 0.0, EPS);
    EXPECT_NEAR(ee.roll,  0.0, EPS);
}

// 旋转矩阵应是正交矩阵：R^T * R = I
TEST(SphericalWrist6DOFTest, FKRotationMatrixIsOrthogonal)
{
    auto arm = make_arm();
    SphericalWrist6DOF::JointState joints{};
    joints.theta[0] = 0.3;
    joints.theta[2] = 1.1;
    joints.theta[4] = -0.7;

    auto ee = arm.forward(joints);

    Eigen::Matrix3d should_be_identity = ee.R.transpose() * ee.R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            EXPECT_NEAR(should_be_identity(i, j), expected, 1e-9)
                << "R^T * R 在 (" << i << ", " << j << ") 处不为 "
                << (i == j ? "1" : "0");
        }
    }
}

// ─── 逆运动学 (IK) 冒烟测试 ──────────────────────────────────────────────────

// IK 至少返回一组解（当前实现为单解）
TEST(SphericalWrist6DOFTest, IKReturnsSolution)
{
    auto arm = make_arm();
    SphericalWrist6DOF::EndEffectorState target{};
    target.x = 0.0;
    target.y = 0.0;
    target.z = TOTAL_Z;
    target.R = Eigen::Matrix3d::Identity();
    target.roll = target.pitch = target.yaw = 0.0;

    auto solutions = arm.inverse(target);
    EXPECT_FALSE(solutions.empty());
}
