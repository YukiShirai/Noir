#include <gtest/gtest.h>

#include <memory>

#include "robot.h"

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
