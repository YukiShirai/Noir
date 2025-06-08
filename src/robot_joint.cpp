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
    out << "  Axis:   " << m_axis.transpose() << "\n";
    out << "  Offset: " << m_offset.transpose() << "\n";
    return out;
}
