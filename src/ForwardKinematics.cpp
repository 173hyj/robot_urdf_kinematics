#include "robot_kinematics/ForwardKinematics.h"

#include <stdexcept>

namespace robot_kinematics {

Eigen::Matrix4d poseToTransform(const Pose& pose) {
    const Eigen::AngleAxisd roll(pose.rpy.x(), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(pose.rpy.y(), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(pose.rpy.z(), Eigen::Vector3d::UnitZ());

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
    transform.block<3, 1>(0, 3) = pose.xyz;
    return transform;
}

Eigen::Matrix4d computeForwardKinematics(const KinematicChain& chain, const std::vector<double>& joint_values) {
    if (joint_values.size() != chain.revoluteJointCount()) {
        throw std::runtime_error("Joint value count does not match revolute joint count");
    }

    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    std::size_t value_index = 0;

    for (const Joint* joint : chain.joints) {
        result *= poseToTransform(joint->origin);

        if (joint->type == JointType::Revolute || joint->type == JointType::Continuous) {
            Eigen::Vector3d axis = joint->axis;
            if (axis.norm() == 0.0) {
                throw std::runtime_error("Joint axis must be non-zero for joint: " + joint->name);
            }
            axis.normalize();

            Eigen::Matrix4d motion = Eigen::Matrix4d::Identity();
            motion.block<3, 3>(0, 0) = Eigen::AngleAxisd(joint_values[value_index], axis).toRotationMatrix();
            result *= motion;
            ++value_index;
        } else if (joint->type == JointType::Prismatic) {
            throw std::runtime_error("Prismatic joints are parsed but not supported by forward kinematics");
        }
    }

    return result;
}

}  // namespace robot_kinematics
