#ifndef ROBOT_H
#define ROBOT_H

#include "robot_joint.h"

namespace noir {

class Robot {
   protected:
    std::string m_name;
    std::vector<std::shared_ptr<Joint>> m_joints;
    // TODO: introduce sensor and controller later
   public:
    explicit Robot(std::string_view name) : m_name(name) {}
    virtual ~Robot() = default;

    virtual void initialize() = 0;
    virtual void sendCommands() = 0;

    // getter
    const std::string& getName() const { return m_name; }
    const std::vector<std::shared_ptr<Joint>>& getJoints() const { return m_joints; }
};

class SingleArm : public Robot {
   public:
    explicit SingleArm(std::string_view name) : Robot(name) {}

    void initialize() override;
    void sendCommands() override;

    virtual Eigen::Matrix4d getEndEffectorTransform(const std::vector<double>& joint_angles) const;
};

class TwoArm : public Robot {
   private:
    std::unique_ptr<SingleArm> m_left_arm;
    std::unique_ptr<SingleArm> m_right_arm;

   public:
    TwoArm(std::string_view name, std::string_view left_arm_name = "left_arm",
           std::string_view right_arm_name = "right_arm");
    void initialize() override;
    void sendCommands() override;
    enum class ArmSide { LEFT, RIGHT };

    virtual Eigen::Matrix4d getEndEffectorTransform(const std::vector<double>& joint_angles,
                                                    const ArmSide& arm_side) const;

    SingleArm* getLeftArm() { return m_left_arm.get(); }
    SingleArm* getRightArm() { return m_right_arm.get(); }
};

}  // namespace noir

#endif  // ROBOT_H
