#include "robot_control_sdk/kinematics/fk_solver.hpp"

#include <cmath>

namespace robot_control_sdk::kinematics{

Matrix4d FKSolver::makeIdentityMatrix(){
    return {{
        {{1.0,0.0,0.0,0.0}},
        {{0.0,1.0,0.0,0.0}},
        {{0.0,0.0,1.0,0.0}},
        {{0.0,0.0,0.0,1.0}}
    }};
}

Matrix4d FKSolver::multiplyMatrix(const Matrix4d& A,const Matrix4d& B){
    Matrix4d C{};

    for (std::size_t i = 0; i < 4; ++i){
        for(std::size_t j = 0; j < 4; ++j){
            C[i][j] = 0.0;
            for(std::size_t k = 0; k < 4; ++k){
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return C;
}

Matrix4d FKSolver::makeDHTransform(double a, double alpha, double d, double theta){
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    return {{
        {{ct, -st * ca, st * sa, a * ct}},
        {{st, ct * ca, -ct * sa, a* st}},
        {{0.0,    sa,  ca   , d}},
        {{0.0,  0.0  ,0.0   ,   1.0}}
    }};
}

Pose FKSolver::compute(const std::array<double, 6>& q) const {
    Pose pose{};
    pose.T = makeIdentityMatrix();

    // UR5 (CB3) 官方标准 DH 参数表: {a, alpha, d}
    const double dh_params[6][3] = {
        {0.0,      M_PI_2,  0.089159},
        {-0.425,   0.0,     0.0},
        {-0.39225, 0.0,     0.0},
        {0.0,      M_PI_2,  0.10915},
        {0.0,     -M_PI_2,  0.09465},
        {0.0,      0.0,     0.0823}
    };

    // 矩阵连乘：T_base_tool = T1 * T2 * T3 * T4 * T5 * T6
    for (int i = 0; i < 6; ++i) {
        Matrix4d Ti = makeDHTransform(dh_params[i][0], dh_params[i][1], dh_params[i][2], q[i]);
        pose.T = multiplyMatrix(pose.T, Ti);
    }

    return pose;
}

}   //  namespace robot_control_sdk::kinematics