#include "robot_control_sdk/kinematics/fk_solver.hpp"

namespace robot_control_sdk::kinematics{

Pose FKSolver::compute(const std::array<double,6>& q) const {
    (void)q;

    Pose pose{};
    pose.T = {{
        {{1.0,0.0,0.0,0.0}},
        {{0.0,1.0,0.0,0.0}},
        {{0.0,0.0,1.0,0.0}},
        {{0.0,0.0,0.0,1.0}}
    }};
    return pose;
}

}