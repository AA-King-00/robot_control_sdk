#include <array>
#include <gtest/gtest.h>
#include "robot_control_sdk/kinematics/fk_solver.hpp"

using namespace robot_control_sdk::kinematics;

TEST(FKSolverTest, ZeroPose){

    FKSolver solver;
    std::array<double,6> q_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Pose result = solver.compute(q_zero);

    EXPECT_NEAR(result.T[0][3], -0.81725, 1e-5);
    EXPECT_NEAR(result.T[1][3], -0.19145, 1e-5);
    EXPECT_NEAR(result.T[2][3], -0.005491, 1e-5);

    EXPECT_DOUBLE_EQ(result.T[3][3], 1.0);
}