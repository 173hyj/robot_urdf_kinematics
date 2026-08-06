#pragma once

#include "robot_kinematics/Model.h"

#include <string>
#include <vector>

namespace robot_kinematics {

struct KinematicChain {
    std::string base_link;
    std::string tip_link;
    std::vector<const Joint*> joints;

    std::size_t revoluteJointCount() const;
};

KinematicChain buildChain(const RobotModel& model, const std::string& base_link, const std::string& tip_link);

}  // namespace robot_kinematics
