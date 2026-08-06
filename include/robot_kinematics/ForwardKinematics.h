#pragma once

#include "robot_kinematics/KinematicChain.h"

#include <Eigen/Dense>

#include <vector>

namespace robot_kinematics {

Eigen::Matrix4d poseToTransform(const Pose& pose);
Eigen::Matrix4d computeForwardKinematics(const KinematicChain& chain, const std::vector<double>& joint_values);

}  // namespace robot_kinematics
