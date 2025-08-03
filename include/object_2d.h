#ifndef OBJECT_2D_H
#define OBJECT_2D_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace noir {
class Object2D {
   protected:
    std::string m_name;
    int m_num_contacts{};

   public:
    explicit Object2D(std::string_view name, int num_contacts) : m_name(name), m_num_contacts(num_contacts) {}
    virtual ~Object2D() = default;
    virtual Eigen::MatrixXd getContactJacobian(const Eigen::VectorXd& angles) const = 0;
    virtual Eigen::Matrix3d getLimitSurface() const = 0;
    virtual std::vector<Eigen::Vector2d> getContactPoints() const = 0;

    // non-abstract method
    virtual std::vector<double> getFrictionConstants() const;
    const std::string& getName() const { return m_name; }
    int getNumContacts() const { return m_num_contacts; }
    virtual std::ostream& print(std::ostream& out) const = 0;
    friend std::ostream& operator<<(std::ostream& out, const Object2D& object2d) {
        return object2d.print(out);
    }
};

class BoxObject2d : public Object2D {
   private:
    double m_width{};
    double m_length{};
    Eigen::Matrix3d m_limit_surface;

   public:
    BoxObject2d(std::string_view name, int num_contacts, double width, double length);
    virtual ~BoxObject2d() override = default;

    virtual Eigen::MatrixXd getContactJacobian(const Eigen::VectorXd& angles) const override;
    virtual Eigen::Matrix3d getLimitSurface() const override;
    virtual std::vector<Eigen::Vector2d> getContactPoints() const override;

    virtual std::ostream& print(std::ostream& out) const override;
};

}  // namespace noir

#endif  // OBJECT_2D_H
