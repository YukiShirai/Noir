#include <gtest/gtest.h>

#include "optimizer_util.h"

using namespace noir;

TEST(ComputeGradientFDTest, QuadraticFunction) {
    auto f = [](const Eigen::VectorXd& x) {
        return x.squaredNorm();
    };
    Eigen::VectorXd x0(2);
    x0 << 1.0, -2.0;
    Eigen::VectorXd grad = compute_gradient_fd(x0, f);
    Eigen::VectorXd expected_grad = 2 * x0;

    // this precision should depend on finite difference epsilon
    EXPECT_TRUE(grad.isApprox(expected_grad, 1e-8));
}

TEST(ComputeGradientFDTest, LinearFUnction) {
    auto f = [](const Eigen::VectorXd& x) {
        return x.sum();  // gradient should be [1, 1, ..., 1]
    };
    int N = 5;
    Eigen::VectorXd x0 = Eigen::VectorXd::Ones(N);
    Eigen::VectorXd grad = compute_gradient_fd(x0, f);
    Eigen::VectorXd expected_grad = Eigen::VectorXd::Ones(N);
    EXPECT_TRUE(grad.isApprox(expected_grad, 1e-8));
}

TEST(BackTrackingLineSearchTest, Constructor) {
    EXPECT_NO_THROW({ BackTrackingLineSearch s(0.3, 0.99); });
    EXPECT_DEATH({ BackTrackingLineSearch strategy(-0.1, 0.9); }, ".*");
    EXPECT_DEATH({ BackTrackingLineSearch strategy(0.3, 1.5); }, ".*");
}

TEST(BackTrackingLineSearchTest, DecreasesFunction) {
    auto f = [](const Eigen::VectorXd& x) {
        return (x.array() - 1.0).square().sum();
    };
    Eigen::VectorXd x(2);
    x << 3.0, 3.0;

    Eigen::VectorXd grad = 2.0 * (x - Eigen::VectorXd::Ones(2));
    Eigen::VectorXd direction = -grad;

    BackTrackingLineSearch line_search(0.3, 0.88);
    double step = line_search.computeStepSize(x, direction, f);

    Eigen::VectorXd x_next = x + step * direction;
    EXPECT_GT(step, 0.0);
    EXPECT_LT(f(x_next), f(x));
}
