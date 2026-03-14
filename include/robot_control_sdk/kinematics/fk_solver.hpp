#pragma once

#include <array>
#include <string>

namespace robot_control_sdk::kinematics{

using Matrix4d = std::array<std::array<double,4>,4>;

struct Pose
{
    Matrix4d T{};
};

struct DHParam {
    double a{0.0};
    double alpha{0.0};
    double d{0.0};
};

class FKSolver{
public:
    FKSolver() = default;
    bool loadDHParams(const std::string& config_file);
    Pose compute(const std::array<double,6>& q) const;

private:
    static Matrix4d makeIdentityMatrix();
    static Matrix4d multiplyMatrix(const Matrix4d& A, const Matrix4d& B);
    static Matrix4d makeDHTransform(double a,double alpha,double d, double theta);

    //作为类的私有变量，永久保存这6个关节的DH参数
    std::array<DHParam,6> dh_params_{};
    bool is_initialized_{false};  //标记是否成功加载了参数
};

}  //namespace robot_control_sdk::kinematics