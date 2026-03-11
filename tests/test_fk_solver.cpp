#include <array>
#include <gtest/gtest.h>

#include "robot_control_sdk/kinematics/fk_solver.hpp"

TEST(FKsolverTest, ReturnIdentityForInitialSkeleton) {
    robot_control_sdk::kinematics::FKSolver solver;
    std::array<double,6> q{0.0,0.0,0.0,0.0,0.0,0.0};

    auto pose = solver.compute(q);

    EXPECT_DOUBLE_EQ(pose.T[0][0],1.0);
    EXPECT_DOUBLE_EQ(pose.T[1][1],1.0);
    EXPECT_DOUBLE_EQ(pose.T[2][2],1.0);
    EXPECT_DOUBLE_EQ(pose.T[3][3],1.0);

    EXPECT_DOUBLE_EQ(pose.T[0][1], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[0][2], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[0][3], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[1][0], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[1][2], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[1][3], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[2][0], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[2][1], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[2][3], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[3][0], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[3][1], 0.0);
    EXPECT_DOUBLE_EQ(pose.T[3][2], 0.0);
}