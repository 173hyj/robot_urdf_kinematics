#include "robot_kinematics/UrdfParser.h"
#include "robot_kinematics/KinematicChain.h"
#include "robot_kinematics/ForwardKinematics.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace robot_kinematics;

std::vector<double> degreesToRadians(const std::vector<double>& degrees) {
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (const double value : degrees) {
        radians.push_back(value * 3.14159265358979323846 / 180.0);
    }
    return radians;
}

std::vector<double> kr90ControllerDegreesToUrdfDegrees(std::vector<double> controller_degrees) {
    controller_degrees.at(1) += 90.0;
    controller_degrees.at(2) -= 90.0;
    return controller_degrees;
}

void test_parse_minimal_revolute_urdf() {
    const std::string xml = R"(
<robot name="mini">
  <link name="base_link"/>
  <link name="tool0"/>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="tool0"/>
    <origin xyz="1 2 3" rpy="0 0 1.57079632679"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="2" velocity="3"/>
  </joint>
</robot>
)";

    const RobotModel model = UrdfParser().parseString(xml);

    assert(model.name == "mini");
    assert(model.links.size() == 2);
    assert(model.joints.size() == 1);
    assert(model.revoluteJointCount() == 1);

    const Joint* joint = model.findJoint("joint_1");
    assert(joint != nullptr);
    assert(joint->type == JointType::Revolute);
    assert(joint->parent_link == "base_link");
    assert(joint->child_link == "tool0");
    assert(joint->origin.xyz.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
    assert(joint->axis.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
    assert(joint->limit.has_value());
    assert(joint->limit->lower == -1.0);
    assert(joint->limit->upper == 1.0);
}

void test_parse_supplied_robot_urdfs() {
    const RobotModel fanuc = UrdfParser().parseFile(
        "/tmp/read_robot_zips/m710/M-710iC50 M-710iC70/urdf/M-710iC50 M-710iC70.STEP.SLDASM.urdf");
    assert(fanuc.findLink("base_link") != nullptr);
    assert(fanuc.findLink("tool0") != nullptr);
    assert(fanuc.revoluteJointCount() == 6);

    const RobotModel kuka =
        UrdfParser().parseFile("/tmp/read_robot_zips/kr90/KR 90 R3100 extra/urdf/KR 90 R3100 extra.urdf");
    assert(kuka.findLink("base_link") != nullptr);
    assert(kuka.findLink("tool0") != nullptr);
    assert(kuka.revoluteJointCount() == 6);
}

void test_rejects_malformed_urdf_root() {
    bool threw = false;
    try {
        (void)UrdfParser().parseString("<not_robot/>");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

void test_builds_supplied_robot_chains() {
    const RobotModel fanuc = UrdfParser().parseFile(
        "/tmp/read_robot_zips/m710/M-710iC50 M-710iC70/urdf/M-710iC50 M-710iC70.STEP.SLDASM.urdf");
    const KinematicChain fanuc_chain = buildChain(fanuc, "base_link", "tool0");
    assert(fanuc_chain.joints.size() == 8);
    assert(fanuc_chain.revoluteJointCount() == 6);
    assert(fanuc_chain.joints.front()->name == "joint_1");
    assert(fanuc_chain.joints.back()->name == "Link_6-tool0");

    const RobotModel kuka =
        UrdfParser().parseFile("/tmp/read_robot_zips/kr90/KR 90 R3100 extra/urdf/KR 90 R3100 extra.urdf");
    const KinematicChain kuka_chain = buildChain(kuka, "base_link", "tool0");
    assert(kuka_chain.joints.size() == 7);
    assert(kuka_chain.revoluteJointCount() == 6);
    assert(kuka_chain.joints.front()->name == "joint_1");
    assert(kuka_chain.joints.back()->name == "Link_6-tool0");
}

void test_zero_angle_forward_kinematics_for_supplied_robots() {
    const RobotModel fanuc = UrdfParser().parseFile(
        "/tmp/read_robot_zips/m710/M-710iC50 M-710iC70/urdf/M-710iC50 M-710iC70.STEP.SLDASM.urdf");
    const KinematicChain fanuc_chain = buildChain(fanuc, "base_link", "tool0");
    const Eigen::Matrix4d fanuc_pose = computeForwardKinematics(fanuc_chain, std::vector<double>{0, 0, 0, 0, 0, 0});
    assert((fanuc_pose.block<3, 1>(0, 3).isApprox(Eigen::Vector3d(1.341, 0.0, 1.605), 1e-9)));

    const RobotModel kuka =
        UrdfParser().parseFile("/tmp/read_robot_zips/kr90/KR 90 R3100 extra/urdf/KR 90 R3100 extra.urdf");
    const KinematicChain kuka_chain = buildChain(kuka, "base_link", "tool0");
    const Eigen::Matrix4d kuka_pose = computeForwardKinematics(kuka_chain, std::vector<double>{0, 0, 0, 0, 0, 0});
    assert((kuka_pose.block<3, 1>(0, 3).isApprox(Eigen::Vector3d(1.965, 0.0, 1.984), 1e-9)));
}

void test_rejects_wrong_joint_value_count() {
    const RobotModel model = UrdfParser().parseString(R"(
<robot name="mini">
  <link name="base_link"/>
  <link name="tool0"/>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="tool0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
)");
    const KinematicChain chain = buildChain(model, "base_link", "tool0");

    bool threw = false;
    try {
        (void)computeForwardKinematics(chain, std::vector<double>{});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

void test_kr90_validation_samples_from_ui_images() {
    const RobotModel kuka =
        UrdfParser().parseFile("/tmp/read_robot_zips/kr90/KR 90 R3100 extra/urdf/KR 90 R3100 extra.urdf");
    const KinematicChain chain = buildChain(kuka, "base_link", "tool0");

    struct Sample {
        std::vector<double> controller_degrees;
        Eigen::Vector3d expected_position_mm;
    };

    const std::vector<Sample> samples = {
        {{4.850, -65.927, 70.028, 3.933, 67.884, -47.191}, {2351.067, -213.202, 1562.578}},
        {{4.850, -118.006, -1.039, 157.303, 67.884, -47.191}, {-1130.167, 18.764, 3092.304}},
        {{70.000, -140.000, 95.000, 30.000, 45.000, 70.000}, {111.680, -529.090, 2518.124}},
        {{50.000, -75.000, 95.000, 30.000, 60.000, 60.000}, {1244.353, -1627.796, 1273.352}},
    };

    for (const Sample& sample : samples) {
        const std::vector<double> urdf_degrees = kr90ControllerDegreesToUrdfDegrees(sample.controller_degrees);
        const Eigen::Matrix4d pose = computeForwardKinematics(chain, degreesToRadians(urdf_degrees));
        const Eigen::Vector3d position_mm = pose.block<3, 1>(0, 3) * 1000.0;
        assert((position_mm.isApprox(sample.expected_position_mm, 1e-3)));
    }
}

int main() {
    test_parse_minimal_revolute_urdf();
    test_parse_supplied_robot_urdfs();
    test_rejects_malformed_urdf_root();
    test_builds_supplied_robot_chains();
    test_zero_angle_forward_kinematics_for_supplied_robots();
    test_rejects_wrong_joint_value_count();
    test_kr90_validation_samples_from_ui_images();
    std::cout << "All tests passed\n";
    return 0;
}
