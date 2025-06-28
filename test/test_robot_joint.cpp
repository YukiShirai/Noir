#include <gtest/gtest.h>

#include "robot_joint.h"

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

TEST(JointInterfaceTest, AxisNormalization) {
    Eigen::Vector3d axis_unnormalized(0, 0, 10);  // not unit length
    Eigen::Vector3d offset(0, 0, 0);

    RevoluteJoint joint("rev_joint", 0, axis_unnormalized, offset);
    Eigen::Matrix4d T = joint.getTransform(M_PI);  // rotate 180°

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Matrix3d expected = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    EXPECT_TRUE(R.isApprox(expected, 1e-9));
}

TEST(JointInterfaceTest, PrintOperatorMethod) {
    {
        RevoluteJoint joint("rev_joint56", 121, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 3));

        std::ostringstream oss;
        oss << joint;

        std::string output = oss.str();
        EXPECT_NE(output.find("rev_joint56"), std::string::npos);
        EXPECT_NE(output.find("121"), std::string::npos);
    }
    {
        PrismaticJoint joint("rev_joint56", 121, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 3));

        std::ostringstream oss;
        oss << joint;

        std::string output = oss.str();
        EXPECT_NE(output.find("rev_joint56"), std::string::npos);
        EXPECT_NE(output.find("121"), std::string::npos);
    }
}

TEST(JointInterfaceTest, NameAndIndexAccessors) {
    RevoluteJoint joint("joint_xyz", 5, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 3));

    EXPECT_EQ(joint.getName(), "joint_xyz");
    EXPECT_EQ(joint.getIndex(), 5);
}

TEST(JointFactoryTest, RevoluteJointTransform) {
    {
        Eigen::Vector3d axis(0, 0, 1);
        Eigen::Vector3d offset(1, 2, 3);

        auto joint = JointFactory::createJoint("revolute", "rev_joint", 0, axis, offset);

        ASSERT_NE(joint, nullptr);
        Eigen::Matrix4d T = joint->getTransform(0.0);

        Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
        expected.block<3, 1>(0, 3) = offset;

        EXPECT_TRUE(T.isApprox(expected, 1e-9));
    }
}

TEST(JointFactoryTest, ThrowsOnUnknownJointType) {
    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Vector3d offset(0, 0, 0);

    EXPECT_THROW(
        { auto joint = JointFactory::createJoint("foobar", "invalid_joint", 99, axis, offset); },
        std::invalid_argument);
}
