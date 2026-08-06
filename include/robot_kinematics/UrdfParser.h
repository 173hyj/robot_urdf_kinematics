#pragma once

#include "robot_kinematics/Model.h"

#include <string>

namespace robot_kinematics {

class UrdfParser {
public:
    RobotModel parseFile(const std::string& path) const;
    RobotModel parseString(const std::string& xml) const;
};

}  // namespace robot_kinematics
