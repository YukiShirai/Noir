#include <gtest/gtest.h>

#include <memory>

#include "object_2d.h"

using namespace noir;

TEST(BoxObject2dTest, CanConstructAndAccessProperties) {
    noir::BoxObject2d box("box", 2, 0.4, 0.2);
    EXPECT_EQ(box.getName(), "box");
    EXPECT_EQ(box.getNumContacts(), 2);
}
TEST(BoxObject2dTest, ReturnsCorrectFrictionConstants) {
    noir::BoxObject2d box("box", 3, 0.4, 0.2);
    auto frictions = box.getFrictionConstants();
    ASSERT_EQ(frictions.size(), 3);
    for (double mu : frictions) {
        EXPECT_DOUBLE_EQ(mu, 0.8);  // your hardcoded value
    }
}
TEST(BoxObject2dTest, StreamOperatorPrintsSomething) {
    noir::BoxObject2d box("box", 2, 0.4, 0.2);
    std::ostringstream oss;
    oss << box;
    std::string output = oss.str();
    EXPECT_NE(output.find("BoxObject2d"), std::string::npos);
    EXPECT_NE(output.find("box"), std::string::npos);
}
TEST(BoxObject2dTest, ContactJacobianReturnsCorrectShape) {
    noir::BoxObject2d box("box", 2, 0.4, 0.2);
    Eigen::VectorXd dummy_angles(2);
    dummy_angles << 0.0, 0.0;  // or any angle values

    auto J = box.getContactJacobian(dummy_angles);
    EXPECT_EQ(J.rows(), 3);
    EXPECT_EQ(J.cols(), 2 * box.getNumContacts());
}

TEST(BoxObject2dTest, ReturnsLimitSurfaceCorrectly) {
    noir::BoxObject2d box("box", 2, 0.4, 0.2);
    Eigen::Matrix3d ls = box.getLimitSurface();
    EXPECT_DOUBLE_EQ(ls(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(ls(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(ls(2, 2), 0.01);
    EXPECT_DOUBLE_EQ(ls(0, 1), 0.0);
}
TEST(BoxObject2dTest, ComputesContactPointsCorrectly) {
    noir::BoxObject2d box("box", 2, 0.4, 0.2);
    auto points = box.getContactPoints();

    ASSERT_EQ(points.size(), 2);

    // First contact point (i = 0)
    EXPECT_NEAR(points[0].x(), -0.4 / 2.0 + 0.4 / 2 / 2, 1e-6);  // center of first half
    EXPECT_NEAR(points[0].y(), -0.2 / 2.0, 1e-6);

    // Second contact point (i = 1)
    EXPECT_NEAR(points[1].x(), -0.4 / 2.0 + 3 * 0.4 / 2 / 2, 1e-6);  // center of second half
    EXPECT_NEAR(points[1].y(), -0.2 / 2.0, 1e-6);
}
