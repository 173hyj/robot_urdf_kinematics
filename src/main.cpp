#include "robot_kinematics/ForwardKinematics.h"
#include "robot_kinematics/KinematicChain.h"
#include "robot_kinematics/UrdfParser.h"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string urdf_path;
    std::string base_link = "base_link";
    std::string tip_link = "tool0";
    std::vector<double> joint_values;
    bool joints_provided = false;
};

void printHelp(std::ostream& out) {
    out << "Usage: robot_kinematics --urdf PATH [--base LINK] [--tip LINK] [--joints q1 q2 ...]\n"
        << "\n"
        << "Reads a URDF model, builds a serial kinematic chain, and prints forward kinematics.\n"
        << "Joint values are radians. If --joints is omitted, zero values are used.\n";
}

double parseDouble(const std::string& value) {
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != value.size()) {
        throw std::runtime_error("Invalid number: " + value);
    }
    return result;
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp(std::cout);
            std::exit(0);
        }
        if (arg == "--urdf") {
            if (++i >= argc) {
                throw std::runtime_error("--urdf requires a path");
            }
            options.urdf_path = argv[i];
        } else if (arg == "--base") {
            if (++i >= argc) {
                throw std::runtime_error("--base requires a link name");
            }
            options.base_link = argv[i];
        } else if (arg == "--tip") {
            if (++i >= argc) {
                throw std::runtime_error("--tip requires a link name");
            }
            options.tip_link = argv[i];
        } else if (arg == "--joints") {
            options.joints_provided = true;
            while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                options.joint_values.push_back(parseDouble(argv[++i]));
            }
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.urdf_path.empty()) {
        throw std::runtime_error("--urdf is required");
    }
    return options;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / 3.14159265358979323846;
}

void printMatrix(const Eigen::Matrix4d& matrix) {
    std::cout << std::fixed << std::setprecision(9);
    for (int row = 0; row < matrix.rows(); ++row) {
        for (int col = 0; col < matrix.cols(); ++col) {
            std::cout << std::setw(15) << matrix(row, col);
        }
        std::cout << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options options = parseOptions(argc, argv);
        const robot_kinematics::RobotModel model = robot_kinematics::UrdfParser().parseFile(options.urdf_path);
        const robot_kinematics::KinematicChain chain =
            robot_kinematics::buildChain(model, options.base_link, options.tip_link);

        if (!options.joints_provided) {
            options.joint_values.assign(chain.revoluteJointCount(), 0.0);
        }

        const Eigen::Matrix4d pose = robot_kinematics::computeForwardKinematics(chain, options.joint_values);

        std::cout << "Robot: " << model.name << '\n';
        std::cout << "Links: " << model.links.size() << '\n';
        std::cout << "Joints: " << model.joints.size() << '\n';
        std::cout << "Chain: " << chain.base_link << " -> " << chain.tip_link << '\n';
        std::cout << "Chain joints:\n";
        for (const robot_kinematics::Joint* joint : chain.joints) {
            std::cout << "  " << joint->name << " [" << robot_kinematics::jointTypeName(joint->type) << "] "
                      << joint->parent_link << " -> " << joint->child_link << " axis=(" << joint->axis.x() << ", "
                      << joint->axis.y() << ", " << joint->axis.z() << ")";
            if (joint->limit.has_value()) {
                std::cout << " limit=[" << joint->limit->lower << ", " << joint->limit->upper << "] rad ["
                          << radiansToDegrees(joint->limit->lower) << ", "
                          << radiansToDegrees(joint->limit->upper) << "] deg";
            }
            std::cout << '\n';
        }

        std::cout << "Forward kinematics transform:\n";
        printMatrix(pose);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        std::cerr << "Run with --help for usage.\n";
        return 1;
    }
}
