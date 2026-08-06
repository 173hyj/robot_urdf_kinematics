#pragma once

#include <Eigen/Dense>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_kinematics {

enum class JointType {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Unknown
};

struct JointLimit {
    double lower = 0.0;
    double upper = 0.0;
    double effort = 0.0;
    double velocity = 0.0;
};

struct Pose {
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
};

struct Link {
    std::string name;
    std::vector<std::string> mesh_filenames;
};

struct Joint {
    std::string name;
    JointType type = JointType::Unknown;
    std::string parent_link;
    std::string child_link;
    Pose origin;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    std::optional<JointLimit> limit;
};

class RobotModel {
public:
    std::string name;
    std::vector<Link> links;
    std::vector<Joint> joints;

    const Link* findLink(const std::string& name) const;
    const Joint* findJoint(const std::string& name) const;
    std::vector<const Joint*> childJointsOf(const std::string& parent_link) const;
    std::size_t revoluteJointCount() const;
};

std::string jointTypeName(JointType type);
JointType jointTypeFromString(const std::string& value);

}  // namespace robot_kinematics
