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
};
}  // namespace noir

#endif  // ROBOT_H
