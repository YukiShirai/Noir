#include "robot_joint.h"

#include <gtest/gtest.h>

using namespace noir;

TEST(JointInerfaceTest, IdentityTransformAtZeroAngle) {
    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Vector3d offset(1, 2, 3);

    RevoluteJoint joint(axis, offset);
    Eigen::Matrix4d T = joint.getTransform(0.0);

    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected.block<3, 1>(0, 3) = offset;

    EXPECT_TRUE(T.isApprox(expected, 1e-9));
}

TEST(JointInerfaceTest, RotationAboutZAxis) {
    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Vector3d offset(0, 0, 0);

    RevoluteJoint joint(axis, offset);
    Eigen::Matrix4d T = joint.getTransform(M_PI / 2);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    Eigen::Matrix3d expected_R;
    expected_R << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    EXPECT_TRUE(R.isApprox(expected_R, 1e-9));
}

TEST(JointInerfaceTest, PolymorphicCall) {
    // TODO: test for other different cases over for loop
    std::unique_ptr<Joint> joint =
        std::make_unique<RevoluteJoint>(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 3));
    // this should return derived class
    Eigen::Matrix4d T = joint->getTransform(0.0);
    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected.block<3, 1>(0, 3) = Eigen::Vector3d{1, 2, 3};

    EXPECT_TRUE(T.isApprox(expected, 1e-9));
}
