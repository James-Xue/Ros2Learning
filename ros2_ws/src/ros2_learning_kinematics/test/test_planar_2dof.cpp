#include <gtest/gtest.h>
#include <cmath>
#include "ros2_learning_kinematics/planar_2dof.hpp"

using ros2_learning_kinematics::Planar2DOF;

// 浮点数比较精度
constexpr double EPS = 1e-9;

// ─── 正运动学 (FK) 测试 ──────────────────────────────────────────────────────

// 零位：theta1=0, theta2=0 → 末端在 (l1+l2, 0)
TEST(Planar2DOFTest, FKZeroAngles)
{
    Planar2DOF arm(1.0, 1.0);
    auto ee = arm.forward({0.0, 0.0});
    EXPECT_NEAR(ee.x, 2.0, EPS);
    EXPECT_NEAR(ee.y, 0.0, EPS);
}

// 肘部 90°：theta1=0, theta2=π/2 → 末端在 (l1, l2)
TEST(Planar2DOFTest, FKElbow90)
{
    Planar2DOF arm(1.0, 1.0);
    auto ee = arm.forward({0.0, M_PI / 2.0});
    EXPECT_NEAR(ee.x, 1.0, EPS);
    EXPECT_NEAR(ee.y, 1.0, EPS);
}

// 肩部 90°：theta1=π/2, theta2=0 → 末端在 (0, l1+l2)
TEST(Planar2DOFTest, FKShoulder90)
{
    Planar2DOF arm(1.0, 1.0);
    auto ee = arm.forward({M_PI / 2.0, 0.0});
    EXPECT_NEAR(ee.x, 0.0, EPS);
    EXPECT_NEAR(ee.y, 2.0, EPS);
}

// 非对称连杆：l1=2, l2=1, 零位 → 末端在 (3, 0)
TEST(Planar2DOFTest, FKAsymmetricLinks)
{
    Planar2DOF arm(2.0, 1.0);
    auto ee = arm.forward({0.0, 0.0});
    EXPECT_NEAR(ee.x, 3.0, EPS);
    EXPECT_NEAR(ee.y, 0.0, EPS);
}

// 折叠 180°：theta1=0, theta2=π → 末端回到原点 (0, 0)
TEST(Planar2DOFTest, FKFolded180)
{
    Planar2DOF arm(1.0, 1.0);
    auto ee = arm.forward({0.0, M_PI});
    EXPECT_NEAR(ee.x, 0.0, EPS);
    EXPECT_NEAR(ee.y, 0.0, EPS);
}

// ─── 逆运动学 (IK) 测试 ──────────────────────────────────────────────────────

// IK 往返验证：将所有解代回 FK，结果应与目标点吻合
TEST(Planar2DOFTest, IKRoundTrip)
{
    Planar2DOF arm(1.0, 1.0);
    Planar2DOF::EndEffectorState target{1.2, 0.8};

    auto solutions = arm.inverse(target);
    ASSERT_FALSE(solutions.empty()) << "期望至少一组 IK 解，但返回空";

    for (const auto & sol : solutions) {
        auto ee = arm.forward(sol);
        EXPECT_NEAR(ee.x, target.x, 1e-9) << "FK(IK 解).x 与目标不符";
        EXPECT_NEAR(ee.y, target.y, 1e-9) << "FK(IK 解).y 与目标不符";
    }
}

// 正常可达点应有两组解（Elbow Up / Elbow Down）
TEST(Planar2DOFTest, IKTwoSolutionsForReachablePoint)
{
    Planar2DOF arm(1.0, 1.0);
    auto solutions = arm.inverse({1.0, 0.5});
    EXPECT_EQ(solutions.size(), 2u) << "期望 Elbow Up / Elbow Down 两组解";
}

// 完全伸展时（末端在最大工作半径上）只有一组解，theta2=0
TEST(Planar2DOFTest, IKFullyExtendedOneSolution)
{
    Planar2DOF arm(1.0, 1.0);
    auto solutions = arm.inverse({2.0, 0.0});
    ASSERT_EQ(solutions.size(), 1u);
    EXPECT_NEAR(solutions[0].theta2, 0.0, 1e-9);
}

// 超出最大工作半径 → 无解
TEST(Planar2DOFTest, IKOutOfReachFar)
{
    Planar2DOF arm(1.0, 1.0);
    auto solutions = arm.inverse({3.0, 0.0});    // max reach = 2.0
    EXPECT_TRUE(solutions.empty()) << "超出工作空间应返回空解";
}

// 非等长连杆：目标点低于最小工作半径 → 无解
TEST(Planar2DOFTest, IKOutOfReachClose)
{
    // l1=2, l2=1 → min_reach = 1.0；目标距原点 0.5，不可达
    Planar2DOF arm(2.0, 1.0);
    auto solutions = arm.inverse({0.5, 0.0});
    EXPECT_TRUE(solutions.empty()) << "低于最小工作半径应返回空解";
}

// IK 多点往返：覆盖不同象限
TEST(Planar2DOFTest, IKRoundTripMultiplePoints)
{
    Planar2DOF arm(1.5, 1.0);
    const std::vector<Planar2DOF::EndEffectorState> targets = {
        {1.5, 0.8},
        {0.5, 1.2},
        {-0.8, 1.0},
        {1.0, -0.5},
    };

    for (const auto & target : targets) {
        auto solutions = arm.inverse(target);
        ASSERT_FALSE(solutions.empty())
            << "目标 (" << target.x << ", " << target.y << ") 应可达";

        for (const auto & sol : solutions) {
            auto ee = arm.forward(sol);
            EXPECT_NEAR(ee.x, target.x, 1e-9);
            EXPECT_NEAR(ee.y, target.y, 1e-9);
        }
    }
}
