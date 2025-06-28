#include "optimizer_util.h"

using namespace noir;

BackTrackingLineSearch::BackTrackingLineSearch(double alpha, double beta) : m_alpha(alpha), m_beta(beta) {
    assert(alpha > 0 && alpha < 0.5 && "alpha must be in (0, 0.5)");
    assert(beta > 0 && beta < 1.0 && "beta must be in (0, 1.0)");
}

double BackTrackingLineSearch::computeStepSize(const Eigen::VectorXd& x, const Eigen::VectorXd& direction,
                                               const std::function<double(const Eigen::VectorXd&)>& f) {
    double t = 1.0;
    double fx = f(x);
    Eigen::VectorXd grad = compute_gradient_fd(x, f);

    while (f(x + t * direction) > fx + m_alpha * t * grad.dot(direction)) {
        t *= m_beta;
    }
    return t;
}

Eigen::VectorXd noir::compute_gradient_fd(const Eigen::VectorXd& x,
                                          const std::function<double(const Eigen::VectorXd&)>& f,
                                          double epsilon) {
    int N = x.size();
    Eigen::VectorXd grad(N);
    for (int i = 0; i < N; i++) {
        Eigen::VectorXd x1 = x;
        x1(i) += epsilon;
        grad(i) = (f(x1) - f(x)) / epsilon;
    }
    return grad;
}
