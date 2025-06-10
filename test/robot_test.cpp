#include "robot.h"

#include <gtest/gtest.h>

#include <memory>

using namespace noir;

TEST(RobotBaseTest, InitialTestRobot) {
    std::unique_ptr<Robot> robot = std::make_unique<SingleArm>("arm 0");

    robot->initialize();

    // TODO later this test should be configuted over yaml file
    EXPECT_EQ(robot->getName(), "arm 0");
    EXPECT_FALSE(robot->getJoints().empty());  // Good practice
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
