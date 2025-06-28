#ifndef OPTIMIZER_UTIL_H
#define OPTIMIZER_UTIL_H

#include "Eigen/Dense"

namespace noir {

class StepStrategy {
   public:
    StepStrategy() = default;
    virtual ~StepStrategy() = default;
    virtual double computeStepSize(const Eigen::VectorXd& x, const Eigen::VectorXd& direction,
                                   const std::function<double(const Eigen::VectorXd&)>& f) = 0;
};

/**
 * Backtracking line search with Armijo condition.
 */
class BackTrackingLineSearch : public StepStrategy {
   private:
    double m_alpha{};
    double m_beta{};

   public:
    BackTrackingLineSearch(double alpha, double beta);
    double computeStepSize(const Eigen::VectorXd& x, const Eigen::VectorXd& direction,
                           const std::function<double(const Eigen::VectorXd&)>& f) override;
};

Eigen::VectorXd compute_gradient_fd(const Eigen::VectorXd& x,
                                    const std::function<double(const Eigen::VectorXd&)>& f,
                                    double epsilon = 1e-8);

}  // namespace noir

#endif  // OPTIMIZER_UTIL_H
