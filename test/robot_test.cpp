#include "robot.h"

#include <gtest/gtest.h>

#include <memory>

using namespace noir;

TEST(RobotBaseTest, InitialTestRobot) {
    std::unique_ptr<Robot> robot = std::make_unique<SingleArm>("arm 0");

    robot->initialize();

    // TODO later this test should be configuted over yaml file
    EXPECT_EQ(robot->getName(), "arm 0");
    EXPECT_FALSE(robot->getJoints().empty());
    EXPECT_EQ(robot->getJoints().size(), 2);
    EXPECT_EQ(robot->getJoints()[0]->getIndex(), 0);
    EXPECT_EQ(robot->getJoints()[1]->getIndex(), 1);
}

TEST(RobotTest, ComputeControlAndSendCommandsViaBase) {
    std::unique_ptr<Robot> robot = std::make_unique<SingleArm>("test_arm");
    robot->initialize();

    std::ostringstream oss;
    std::streambuf* original_buf = std::cout.rdbuf();
    std::cout.rdbuf(oss.rdbuf());

    robot->sendCommands();

    std::cout.rdbuf(original_buf);

    std::string output = oss.str();
    EXPECT_NE(output.find("Sending command to joint"), std::string::npos);
}

TEST(RobotTest, FK) {
    SingleArm arm("test_arm");
    arm.initialize();

    std::vector<double> q_zero{0, 0};
    auto T_zero = arm.getEndEffectorTransform(q_zero);

    // First joint offset: (0, 0, 0), second: (0, 0, 1)
    // So with zero angles, expect translation (0, 0, 1)
    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected(2, 3) = 1.0;

    EXPECT_TRUE(T_zero.isApprox(expected, 1e-9));

    // then cosider non-zero angle
    std::vector<double> q = {M_PI / 2, 0.0};
    Eigen::Matrix4d T = arm.getEndEffectorTransform(q);
    EXPECT_NEAR(T(0, 3), 0.0, 1e-9);
    EXPECT_NEAR(T(1, 3), 0.0, 1e-9);
    EXPECT_NEAR(T(2, 3), 1.0, 1e-9);  // end-effector still 1m above

    Eigen::Matrix3d R_actual = T.block<3, 3>(0, 0);
    Eigen::Matrix3d R_expected;
    R_expected << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    EXPECT_TRUE(R_actual.isApprox(R_expected, 1e-9));
}

TEST(RobotTest, FKThrowsOnWrongSize) {
    SingleArm arm("test_arm");
    arm.initialize();

    std::vector<double> wrong_size = {0.0};  // only 1 angle for 2 joints

    EXPECT_THROW(arm.getEndEffectorTransform(wrong_size), std::runtime_error);
}

TEST(TwoRobotTest, InitialTestRobot) {
    std::shared_ptr<Robot> robot = std::make_shared<TwoArm>("bimanual");

    robot->initialize();
    // to access child method, you have to do dynamic_cast
    std::shared_ptr<TwoArm> bimanual_robot = std::dynamic_pointer_cast<TwoArm>(robot);

    EXPECT_EQ(robot->getName(), "bimanual");
    EXPECT_EQ(bimanual_robot->getLeftArm()->getName(), "left_arm");
    EXPECT_EQ(bimanual_robot->getRightArm()->getName(), "right_arm");

    EXPECT_FALSE(robot->getJoints().empty());

    EXPECT_NE(bimanual_robot.get(), nullptr);

    EXPECT_GT(bimanual_robot->getLeftArm()->getJoints().size(), 0);
    EXPECT_GT(bimanual_robot->getRightArm()->getJoints().size(), 0);
    EXPECT_EQ(robot->getJoints().size(), bimanual_robot->getLeftArm()->getJoints().size() +
                                             bimanual_robot->getRightArm()->getJoints().size());
}

TEST(TwoRobotTest, ComputeControlAndSendCommandViaTwoArm) {
    TwoArm robot("bimanual");
    robot.initialize();

    std::ostringstream oss;
    std::streambuf* original_buf = std::cout.rdbuf();
    std::cout.rdbuf(oss.rdbuf());

    robot.sendCommands();

    std::cout.rdbuf(original_buf);  // Restore original

    std::string output = oss.str();
    EXPECT_NE(output.find("Sending command to joint"), std::string::npos);
}

TEST(TwoRobotTest, AccessorReturnSameInstances) {
    TwoArm robot("bimanual");
    robot.initialize();

    SingleArm* left_ptr_1 = robot.getLeftArm();
    SingleArm* left_ptr_2 = robot.getLeftArm();
    EXPECT_EQ(left_ptr_1, left_ptr_2);

    SingleArm* right_ptr_1 = robot.getRightArm();
    SingleArm* right_ptr_2 = robot.getRightArm();
    EXPECT_EQ(right_ptr_1, right_ptr_2);
}

TEST(TwoArmTest, FKLeftArmZeroConfig) {
    TwoArm robot("bimanual");
    robot.initialize();

    std::vector<double> q = {0.0, 0.0};

    Eigen::Matrix4d T = robot.getEndEffectorTransform(q, TwoArm::ArmSide::LEFT);

    // Only second joint has offset (0, 0, 1), so total translation is (0, 0, 1)
    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected(2, 3) = 1.0;

    EXPECT_TRUE(T.isApprox(expected, 1e-9));
}

TEST(TwoArmTest, FKRightArmWithRotation) {
    TwoArm robot("bimanual");
    robot.initialize();

    std::vector<double> q = {M_PI / 2, 0.0};

    Eigen::Matrix4d T = robot.getEndEffectorTransform(q, TwoArm::ArmSide::RIGHT);

    // Translation should still be (0, 0, 1)
    EXPECT_NEAR(T(0, 3), 0.0, 1e-9);
    EXPECT_NEAR(T(1, 3), 0.0, 1e-9);
    EXPECT_NEAR(T(2, 3), 1.0, 1e-9);

    // Rotation should be 90 degrees about Z
    Eigen::Matrix3d R_expected;
    R_expected << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d R_actual = T.block<3, 3>(0, 0);
    EXPECT_TRUE(R_actual.isApprox(R_expected, 1e-9));
}

TEST(TwoArmTest, FKThrowsOnWrongJointSize) {
    TwoArm robot("bimanual");
    robot.initialize();

    std::vector<double> q_bad = {0.0};  // too short

    EXPECT_THROW(robot.getEndEffectorTransform(q_bad, TwoArm::ArmSide::LEFT), std::runtime_error);
    EXPECT_THROW(robot.getEndEffectorTransform(q_bad, TwoArm::ArmSide::RIGHT), std::runtime_error);
}

TEST(TwoArmTest, FKThrowsOnInvalidArmSide) {
    TwoArm robot("bimanual");
    robot.initialize();

    std::vector<double> q = {0.0, 0.0};

    // Use static_cast to simulate bad enum
    TwoArm::ArmSide invalid_side = static_cast<TwoArm::ArmSide>(-1);
    EXPECT_THROW(robot.getEndEffectorTransform(q, invalid_side), std::invalid_argument);
}

class RobotSensorTest : public ::testing::Test {
   protected:
    SingleArm robot{"teet_arm"};

    void SetUp() override {
        robot.initialize();
        robot.addSensor(std::make_shared<ForceSensor>("ft"));
        robot.addSensor(std::make_shared<VisionSensor>("arm_camera"));
    };
};

TEST_F(RobotSensorTest, SensorReadAndMeasurementByName) {
    auto fs = robot.getSensorByName("ft");
    fs->read();
    SensorData fdata = fs->getMeasurement();
    EXPECT_TRUE(std::holds_alternative<Eigen::Vector3d>(fdata));
    Eigen::Vector3d f = std::get<Eigen::Vector3d>(fdata);
    EXPECT_EQ(f.size(), 3);

    auto cam = robot.getSensorByName("arm_camera");
    cam->read();
    SensorData cdata = cam->getMeasurement();
    EXPECT_TRUE(std::holds_alternative<int>(cdata));
    int frame = std::get<int>(cdata);
    EXPECT_GE(frame, 0);
}

TEST_F(RobotSensorTest, GetNonExistentSensorThrows) {
    EXPECT_THROW(robot.getSensorByName("nonexistent_sensor"), std::invalid_argument);
}
