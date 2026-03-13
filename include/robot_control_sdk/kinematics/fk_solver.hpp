#pragma once

#include <array>

namespace robot_control_sdk::kinematics{

using Matrix4d = std::array<std::array<double,4>,4>;



struct Pose
{
    Matrix4d T{};

};

class FKSolver{
public:
    FKSolver() = default;
    Pose compute(const std::array<double,6>& q) const;

private:
    static Matrix4d makeIdentityMatrix();
    static Matrix4d multiplyMatrix(const Matrix4d& A, const Matrix4d& B);
    static Matrix4d makeDHTransform(double a,double alpha,double d, double theta);
};

}  //namespace robot_control_sdk::kinematics