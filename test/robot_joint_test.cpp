#include "robot_joint.h"

#include <gtest/gtest.h>

using namespace noir;

TEST(JointInerfaceTest, IdentityTransformAtZeroAngle) {
    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Vector3d offset(1, 2, 3);

    RevoluteJoint joint("rev_joint", 0, axis, offset);
    Eigen::Matrix4d T = joint.getTransform(0.0);

    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected.block<3, 1>(0, 3) = offset;

    EXPECT_TRUE(T.isApprox(expected, 1e-9));
}

TEST(JointInterfaceTest, IndentityTransformAtZeroDisplacement) {
    Eigen::Vector3d axis(1, 0, 0);
    Eigen::Vector3d offset(1, 2, 3);

    PrismaticJoint joint("prism_joint", 1, axis, offset);
    Eigen::Matrix4d T = joint.getTransform(0.0);

    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected.block<3, 1>(0, 3) = offset;

    EXPECT_TRUE(T.isApprox(expected, 1e-9));
}

TEST(JointInerfaceTest, RotationAboutZAxis) {
    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Vector3d offset(0, 0, 0);

    RevoluteJoint joint("rev_joint", 0, axis, offset);
    Eigen::Matrix4d T = joint.getTransform(M_PI / 2);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    Eigen::Matrix3d expected_R;
    expected_R << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    EXPECT_TRUE(R.isApprox(expected_R, 1e-9));
}

TEST(JointInerfaceTest, TranslationAlongAxis) {
    Eigen::Vector3d axis(1, 0, 0);
    Eigen::Vector3d offset(0, 0, 0);

    PrismaticJoint joint("prism_joint", 1, axis, offset);
    Eigen::Matrix4d T = joint.getTransform(2.0);

    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected.block<3, 1>(0, 3) = 2.0 * axis.normalized();

    EXPECT_TRUE(T.isApprox(expected, 1e-9));
}

TEST(JointInterfaceTest, PolymorphicCall) {
    // RevoluteJoint test
    {
        std::unique_ptr<Joint> joint = std::make_unique<RevoluteJoint>(
            "rev_joint", 0, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 3));

        Eigen::Matrix4d T = joint->getTransform(0.0);
        Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
        expected.block<3, 1>(0, 3) = Eigen::Vector3d{1, 2, 3};

        EXPECT_TRUE(T.isApprox(expected, 1e-9));
    }

    // PrismaticJoint test
    {
        std::unique_ptr<Joint> joint = std::make_unique<PrismaticJoint>(
            "prim_joint", 4, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(9, 10, 11));

        Eigen::Matrix4d T = joint->getTransform(0.0);
        Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
        expected.block<3, 1>(0, 3) = Eigen::Vector3d{9, 10, 11};

        EXPECT_TRUE(T.isApprox(expected, 1e-9));
    }
}
