#include "robot_kinematics/KinematicChain.h"

#include <algorithm>
#include <functional>
#include <unordered_set>
#include <stdexcept>

namespace robot_kinematics {

std::size_t KinematicChain::revoluteJointCount() const {
    std::size_t count = 0;
    for (const Joint* joint : joints) {
        if (joint->type == JointType::Revolute || joint->type == JointType::Continuous) {
            ++count;
        }
    }
    return count;
}

KinematicChain buildChain(const RobotModel& model, const std::string& base_link, const std::string& tip_link) {
    if (model.findLink(base_link) == nullptr) {
        throw std::runtime_error("Base link not found: " + base_link);
    }
    if (model.findLink(tip_link) == nullptr) {
        throw std::runtime_error("Tip link not found: " + tip_link);
    }

    KinematicChain chain;
    chain.base_link = base_link;
    chain.tip_link = tip_link;

    std::unordered_set<std::string> visited;

    std::function<bool(const std::string&)> search = [&](const std::string& current) {
        if (current == tip_link) {
            return true;
        }
        if (!visited.insert(current).second) {
            return false;
        }

        std::vector<const Joint*> children = model.childJointsOf(current);
        std::sort(children.begin(), children.end(), [&](const Joint* left, const Joint* right) {
            if (left->child_link == tip_link) {
                return true;
            }
            if (right->child_link == tip_link) {
                return false;
            }
            return left->name < right->name;
        });

        for (const Joint* joint : children) {
            chain.joints.push_back(joint);
            if (search(joint->child_link)) {
                return true;
            }
            chain.joints.pop_back();
        }

        return false;
    };

    if (!search(base_link)) {
        throw std::runtime_error("Cannot resolve chain from " + base_link + " to " + tip_link);
    }

    return chain;
}

}  // namespace robot_kinematics
