
#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H

#include <Eigen/Dense>

namespace noir {

class Joint {
   public:
    virtual ~Joint() = default;
    virtual Eigen::Matrix4d getTransform(double q) const = 0;
    virtual std::ostream& print(std::ostream& out) const = 0;
    friend std::ostream& operator<<(std::ostream& out, const Joint& joint) { return joint.print(out); }
};

class RevoluteJoint : public Joint {
   private:
    Eigen::Vector3d m_axis;
    Eigen::Vector3d m_offset;

   public:
    RevoluteJoint(const Eigen::Vector3d& axis, const Eigen::Vector3d& offset)
        : m_axis(axis.normalized()), m_offset(offset) {}
    Eigen::Matrix4d getTransform(double q) const override;

    std::ostream& print(std::ostream& out) const override;
};

}  // namespace noir

#endif  // ROBOT_JOINT_H
