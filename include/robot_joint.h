
#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H

#include <Eigen/Dense>

namespace noir {
class Joint {
   protected:
    std::string m_name;
    int m_index{};

   public:
    Joint(std::string_view name, int index) : m_name(name), m_index(index) {}
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
    RevoluteJoint(std::string_view name, int index, const Eigen::Vector3d& axis,
                  const Eigen::Vector3d& offset)
        : Joint(name, index), m_axis(axis.normalized()), m_offset(offset) {}
    Eigen::Matrix4d getTransform(double q) const override;

    std::ostream& print(std::ostream& out) const override;
};

class PrismaticJoint : public Joint {
   private:
    Eigen::Vector3d m_axis;
    Eigen::Vector3d m_offset;

   public:
    PrismaticJoint(std::string_view name, int index, const Eigen::Vector3d& axis,
                   const Eigen::Vector3d& offset)
        : Joint(name, index), m_axis(axis.normalized()), m_offset(offset) {}

    Eigen::Matrix4d getTransform(double q) const override;
    std::ostream& print(std::ostream& out) const override;
};

}  // namespace noir

#endif  // ROBOT_JOINT_H
