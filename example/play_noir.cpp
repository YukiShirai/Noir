#include <iostream>

#include "robot.h"
#include "robot_joint.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    std::cout << "Hello, Noir!" << std::endl;
    MatrixXd m = MatrixXd::Random(3, 3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    std::cout << "m =" << std::endl << m << std::endl;
    VectorXd v(3);
    v << 1, 2, 3;
    std::cout << "m * v =" << std::endl << m * v << std::endl;

    noir::RevoluteJoint joint =
        noir::RevoluteJoint("rev_joint", 0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    std::cout << joint << std::endl;

    noir::SingleArm robot("one_arm");
    robot.initialize();
    robot.sendCommands();

    std::vector<double> angles = {M_PI / 4, -M_PI / 2};

    // Compute FK
    Eigen::Matrix4d T_EE = robot.getEndEffectorTransform(angles);
    std::cout << "End-effector pose:\n" << T_EE << std::endl;

    return 0;
}
