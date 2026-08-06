#include "robot_kinematics/Model.h"

#include <algorithm>

namespace robot_kinematics {

const Link* RobotModel::findLink(const std::string& link_name) const {
    const auto it = std::find_if(links.begin(), links.end(), [&](const Link& link) {
        return link.name == link_name;
    });
    return it == links.end() ? nullptr : &*it;
}

const Joint* RobotModel::findJoint(const std::string& joint_name) const {
    const auto it = std::find_if(joints.begin(), joints.end(), [&](const Joint& joint) {
        return joint.name == joint_name;
    });
    return it == joints.end() ? nullptr : &*it;
}

std::vector<const Joint*> RobotModel::childJointsOf(const std::string& parent_link) const {
    std::vector<const Joint*> result;
    for (const Joint& joint : joints) {
        if (joint.parent_link == parent_link) {
            result.push_back(&joint);
        }
    }
    return result;
}

std::size_t RobotModel::revoluteJointCount() const {
    return static_cast<std::size_t>(std::count_if(joints.begin(), joints.end(), [](const Joint& joint) {
        return joint.type == JointType::Revolute || joint.type == JointType::Continuous;
    }));
}

std::string jointTypeName(JointType type) {
    switch (type) {
        case JointType::Fixed:
            return "fixed";
        case JointType::Revolute:
            return "revolute";
        case JointType::Continuous:
            return "continuous";
        case JointType::Prismatic:
            return "prismatic";
        case JointType::Unknown:
            return "unknown";
    }
    return "unknown";
}

JointType jointTypeFromString(const std::string& value) {
    if (value == "fixed") {
        return JointType::Fixed;
    }
    if (value == "revolute") {
        return JointType::Revolute;
    }
    if (value == "continuous") {
        return JointType::Continuous;
    }
    if (value == "prismatic") {
        return JointType::Prismatic;
    }
    return JointType::Unknown;
}

}  // namespace robot_kinematics
