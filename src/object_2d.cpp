#include "object_2d.h"

namespace noir {

std::vector<double> Object2D::getFrictionConstants() const {
    // TODO: magic number for friction!! Should be in yaml
    return std::vector<double>(m_num_contacts, 0.8);
}

BoxObject2d::BoxObject2d(std::string_view name, int num_contacts, double width, double length)
    : Object2D(name, num_contacts), m_width(width), m_length(length) {
    m_limit_surface.setZero();
    // TODO: magic number for now
    m_limit_surface(0, 0) = 1.0;   // fx
    m_limit_surface(1, 1) = 1.0;   // fy
    m_limit_surface(2, 2) = 0.01;  // torque
}

std::vector<Eigen::Vector2d> BoxObject2d::getContactPoints() const {
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < m_num_contacts; i++) {
        double x = -m_width / 2.0 + (i + 0.5) * m_width / m_num_contacts;
        double y = -m_length / 2.0;
        points.emplace_back(x, y);
    }
    return points;
}

Eigen::Matrix3d BoxObject2d::getLimitSurface() const {
    return m_limit_surface;
}

Eigen::MatrixXd BoxObject2d::getContactJacobian(const Eigen::VectorXd& angles) const {
    if (angles.size() != m_num_contacts) {
        throw std::invalid_argument("size of angle does not match the number of contacts");
    }
    const auto contact_points = getContactPoints();
    Eigen::MatrixXd J(3, 2 * m_num_contacts);

    J.setZero();
    // insert contact jacobian over n points
    for (int i = 0; i < m_num_contacts; i++) {
        const auto& p = contact_points[i];
        double theta = angles(i);

        Eigen::Vector2d n(std::cos(theta), std::sin(theta));
        Eigen::Vector2d d(-std::sin(theta), std::cos(theta));
        J(0, 2 * i + 0) = n(0);
        J(1, 2 * i + 0) = n(1);
        J(2, 2 * i + 0) = p.x() * n(1) - p.y() * n(0);

        // Tangential force
        J(0, 2 * i + 1) = d(0);
        J(1, 2 * i + 1) = d(1);
        J(2, 2 * i + 1) = p.x() * d(1) - p.y() * d(0);
    }
    return J;
}

std::ostream& BoxObject2d::print(std::ostream& out) const {
    out << "BoxObject2d(name=" << m_name << ", contacts=" << m_num_contacts << ", width=" << m_width
        << ", height=" << m_length << ")";
    return out;
}

}  // namespace noir
