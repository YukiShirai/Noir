#include "robot_joint.h"

using namespace noir;

Eigen::Matrix4d RevoluteJoint::getTransform(double q) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    Eigen::Matrix3d R = Eigen::AngleAxisd(q, m_axis).toRotationMatrix();

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = m_offset;

    return T;
}

std::ostream& RevoluteJoint::print(std::ostream& out) const {
    out << "RevoluteJoint:\n";
    out << "  Name and Index:   " << m_name << ", " << m_index << "\n";
    out << "  Axis:   " << m_axis.transpose() << "\n";
    out << "  Offset: " << m_offset.transpose() << "\n";
    return out;
}

Eigen::Matrix4d PrismaticJoint::getTransform(double q) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d trans = m_offset + q * m_axis;
    T.block<3, 1>(0, 3) = trans;
    return T;
}

std::ostream& PrismaticJoint::print(std::ostream& out) const {
    out << "RevoluteJoint:\n";
    out << "  Name and Index:   " << m_name << ", " << m_index << "\n";
    out << "  Axis:   " << m_axis.transpose() << "\n";
    out << "  Offset: " << m_offset.transpose() << "\n";
    return out;
}

std::shared_ptr<Joint> noir::JointFactory::createJoint(std::string_view type, std::string_view name,
                                                       int index, const Eigen::Vector3d& axis,
                                                       const Eigen::Vector3d& offset) {
    // C++ does not support switch for std::string
    if (type == "revolute") {
        return std::make_shared<RevoluteJoint>(name, index, axis, offset);
    } else if (type == "prismatic") {
        return std::make_shared<RevoluteJoint>(name, index, axis, offset);
    } else {
        throw std::invalid_argument("JointFactory::createJoint - unknown joint type");
    }
}
