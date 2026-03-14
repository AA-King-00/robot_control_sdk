#include "robot_control_sdk/kinematics/fk_solver.hpp"

#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>

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

//加载yaml文件
bool FKSolver::loadDHParams(const std::string& config_file){
    try{
        //1. 加载文件
        YAML::Node config = YAML::LoadFile(config_file);

        //2. 检查节点是否存在
        if (!config["ur5_dh"]){
            std::cerr << "[Error] Cannot find 'ur5_dh' node in" << config_file << std::endl;
            return false;
        }

        YAML::Node dh_node  = config["ur5_dh"];

        //3. 检查关节数量是否正确
        if(dh_node.size() != 6){
            std::cerr << "[Error] Expected 6 joint , but got " << dh_node.size() << std::endl;
            return false;
        }

        //4. 将YAML 数据读入私有成员变量 dh_params_ 中
        for(std::size_t i = 0 ; i < 6 ; ++i){
            dh_params_[i].a = dh_node[i]["a"].as<double>();
            dh_params_[i].alpha = dh_node[i]["alpha"].as<double>();
            dh_params_[i].d = dh_node[i]["d"].as<double>();
        }

        is_initialized_ = true;
        return true;
    }catch (const YAML::Exception& e){
        std::cerr << " [YAML Error] " << e.what() << std::endl;
        return false;
    }
}

Pose FKSolver::compute(const std::array<double, 6>& q) const {
    Pose pose{};
    pose.T = makeIdentityMatrix();

   // 如果没有加载配置文件，立即警告并返回单位矩阵
   if (!is_initialized_){
    std::cerr << "Warning DH parameters not loaded! Call loadDHParams() first." << std::endl;
    return pose;
   }

    // 矩阵连乘：T_base_tool = T1 * T2 * T3 * T4 * T5 * T6
    for (int i = 0; i < 6; ++i) {
        Matrix4d Ti = makeDHTransform(dh_params_[i].a, dh_params_[i].alpha, dh_params_[i].d, q[i]);
        pose.T = multiplyMatrix(pose.T, Ti);
    }

    return pose;
}

}   //  namespace robot_control_sdk::kinematics