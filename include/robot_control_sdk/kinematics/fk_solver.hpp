#pragma once

#include <array>

namespace robot_control_sdk::kinematics{

struct Pose
{
    std::array<std::array<double,4>,4> T{};

};

class FKSolver
{
public:
    FKSolver() = default;
    Pose compute(const std::array<double,6>& q) const;
};

}