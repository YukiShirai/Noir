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

TwoArm::TwoArm(std::string_view name, std::string_view left_arm_name, std::string_view right_arm_name)
    : Robot(name) {
    m_left_arm = std::make_unique<SingleArm>(left_arm_name);
    m_right_arm = std::make_unique<SingleArm>(right_arm_name);
}

void TwoArm::initialize() {
    m_left_arm->initialize();
    m_right_arm->initialize();

    m_joints.clear();
    m_joints.insert(m_joints.end(), m_left_arm->getJoints().begin(), m_left_arm->getJoints().end());
    m_joints.insert(m_joints.end(), m_right_arm->getJoints().begin(), m_right_arm->getJoints().end());
}

void TwoArm::sendCommands() {
    m_left_arm->sendCommands();
    m_right_arm->sendCommands();
}
