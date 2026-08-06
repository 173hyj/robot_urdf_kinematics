#include "robot_kinematics/UrdfParser.h"

#include <tinyxml2.h>

#include <array>
#include <sstream>
#include <stdexcept>

namespace robot_kinematics {
namespace {

std::string requireAttribute(const tinyxml2::XMLElement& element, const char* name) {
    const char* value = element.Attribute(name);
    if (value == nullptr || std::string(value).empty()) {
        throw std::runtime_error(std::string("Missing required attribute '") + name + "'");
    }
    return value;
}

double parseDouble(const char* value, const std::string& context) {
    if (value == nullptr || std::string(value).empty()) {
        return 0.0;
    }
    std::size_t consumed = 0;
    const std::string text(value);
    try {
        const double result = std::stod(text, &consumed);
        if (consumed != text.size()) {
            throw std::runtime_error("trailing input");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid numeric value for " + context + ": " + text);
    }
}

Eigen::Vector3d parseVector3(const char* value, const Eigen::Vector3d& default_value, const std::string& context) {
    if (value == nullptr || std::string(value).empty()) {
        return default_value;
    }

    std::istringstream stream(value);
    std::array<double, 3> values{};
    if (!(stream >> values[0] >> values[1] >> values[2])) {
        throw std::runtime_error("Invalid vector value for " + context + ": " + value);
    }

    std::string extra;
    if (stream >> extra) {
        throw std::runtime_error("Invalid vector value for " + context + ": " + value);
    }

    return Eigen::Vector3d(values[0], values[1], values[2]);
}

Pose parseOrigin(const tinyxml2::XMLElement* origin) {
    Pose pose;
    if (origin == nullptr) {
        return pose;
    }
    pose.xyz = parseVector3(origin->Attribute("xyz"), Eigen::Vector3d::Zero(), "origin xyz");
    pose.rpy = parseVector3(origin->Attribute("rpy"), Eigen::Vector3d::Zero(), "origin rpy");
    return pose;
}

void collectMeshFilenames(const tinyxml2::XMLElement* element, std::vector<std::string>& filenames) {
    if (element == nullptr) {
        return;
    }
    if (std::string(element->Name()) == "mesh") {
        if (const char* filename = element->Attribute("filename")) {
            filenames.emplace_back(filename);
        }
    }
    for (const tinyxml2::XMLElement* child = element->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        collectMeshFilenames(child, filenames);
    }
}

JointLimit parseLimit(const tinyxml2::XMLElement& limit) {
    JointLimit result;
    result.lower = parseDouble(limit.Attribute("lower"), "limit lower");
    result.upper = parseDouble(limit.Attribute("upper"), "limit upper");
    result.effort = parseDouble(limit.Attribute("effort"), "limit effort");
    result.velocity = parseDouble(limit.Attribute("velocity"), "limit velocity");
    return result;
}

RobotModel parseDocument(tinyxml2::XMLDocument& document) {
    const tinyxml2::XMLElement* root = document.RootElement();
    if (root == nullptr || std::string(root->Name()) != "robot") {
        throw std::runtime_error("URDF root element must be <robot>");
    }

    RobotModel model;
    model.name = requireAttribute(*root, "name");

    for (const tinyxml2::XMLElement* element = root->FirstChildElement("link"); element != nullptr;
         element = element->NextSiblingElement("link")) {
        Link link;
        link.name = requireAttribute(*element, "name");
        collectMeshFilenames(element, link.mesh_filenames);
        model.links.push_back(std::move(link));
    }

    for (const tinyxml2::XMLElement* element = root->FirstChildElement("joint"); element != nullptr;
         element = element->NextSiblingElement("joint")) {
        Joint joint;
        joint.name = requireAttribute(*element, "name");
        joint.type = jointTypeFromString(requireAttribute(*element, "type"));
        joint.origin = parseOrigin(element->FirstChildElement("origin"));

        const tinyxml2::XMLElement* parent = element->FirstChildElement("parent");
        const tinyxml2::XMLElement* child = element->FirstChildElement("child");
        if (parent == nullptr || child == nullptr) {
            throw std::runtime_error("Joint '" + joint.name + "' must define parent and child links");
        }
        joint.parent_link = requireAttribute(*parent, "link");
        joint.child_link = requireAttribute(*child, "link");

        const tinyxml2::XMLElement* axis = element->FirstChildElement("axis");
        joint.axis = parseVector3(axis == nullptr ? nullptr : axis->Attribute("xyz"), Eigen::Vector3d::UnitX(),
                                  "axis xyz");

        if (const tinyxml2::XMLElement* limit = element->FirstChildElement("limit")) {
            joint.limit = parseLimit(*limit);
        }

        model.joints.push_back(std::move(joint));
    }

    return model;
}

}  // namespace

RobotModel UrdfParser::parseFile(const std::string& path) const {
    tinyxml2::XMLDocument document;
    const tinyxml2::XMLError error = document.LoadFile(path.c_str());
    if (error != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Unable to load URDF file: " + path);
    }
    return parseDocument(document);
}

RobotModel UrdfParser::parseString(const std::string& xml) const {
    tinyxml2::XMLDocument document;
    const tinyxml2::XMLError error = document.Parse(xml.c_str(), xml.size());
    if (error != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Unable to parse URDF XML string");
    }
    return parseDocument(document);
}

}  // namespace robot_kinematics
