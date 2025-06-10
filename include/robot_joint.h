#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H

#include <Eigen/Dense>
#include <memory>

namespace noir {
class Joint {
   protected:
    std::string m_name;
    int m_index{};
    double m_position{};
    double m_velocity{};
    double m_torque{};

   public:
    Joint(std::string_view name, int index) : m_name(name), m_index(index) {}
    virtual ~Joint() = default;
    virtual Eigen::Matrix4d getTransform(double q) const = 0;
    virtual std::ostream& print(std::ostream& out) const = 0;
    friend std::ostream& operator<<(std::ostream& out, const Joint& joint) { return joint.print(out); }

    // getter
    std::string getName() const { return m_name; }
    int getIndex() const { return m_index; }
    double getPosition() const { return m_position; }
    double getVelocity() const { return m_velocity; }
    double getTorque() const { return m_torque; }
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

// Factory for Joint to centralize the creation using single class
class JointFactory {
   public:
    // since we don't keep states, we use static method here so users don't need to instantiate this class
    static std::shared_ptr<Joint> createJoint(std::string_view type, std::string_view name, int index,
                                              const Eigen::Vector3d& axis, const Eigen::Vector3d& offset);
};

}  // namespace noir

#endif  // ROBOT_JOINT_H
