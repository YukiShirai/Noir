#include "robot.h"

#include <iostream>

using namespace noir;

void SingleArm::initialize() {
    // TODO: get from yaml file
    std::vector<JointSpec> joint_specs = {
        {"revolute", "base", 0, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{0, 0, 0}},
        {"revolute", "elbow", 1, Eigen::Vector3d{0, 1, 0}, Eigen::Vector3d{0, 0, 1}},
    };

    for (const auto& spec : joint_specs) {
        m_joints.push_back(
            JointFactory::createJoint(spec.type, spec.name, spec.index, spec.axis, spec.offset));
    }
    // TODO: initialize controller and sensor as well.
}

void SingleArm::sendCommands() {
    for (const auto& joint : m_joints) {
        std::cout << "Sending command to joint: " << *joint << std::endl;
    }
}
