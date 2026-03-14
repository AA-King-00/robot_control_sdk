#include <array>
#include <iostream>

#include "robot_control_sdk/kinematics/fk_solver.hpp"

int main(){
    robot_control_sdk::kinematics::FKSolver solver;

    if(!solver.loadDHParams("../config/ur5_dh.yaml")){
        std::cerr << "Failed to load DH parameters!" << std::endl;
        return -1;
    }
    std::array<double,6> q{0.0,0.0,0.0,0.0,0.0,0.0};

    auto pose = solver.compute(q);

    std::cout << "FK result:\n";
    for (const auto& row :pose.T)
    {
        for (double v : row){
            std::cout << v <<" ";
        }
        std::cout << "\n";
    }

    return 0;
}