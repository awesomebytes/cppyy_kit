#include <Eigen/Core>
#include <Eigen/Cholesky>

// A damped joint increment for a 6D task and a seven-joint arm.
void joint_step(const double* jacobian, const double* delta,
                double damping, double* output) {
    using Jacobian = Eigen::Matrix<double, 6, 7, Eigen::RowMajor>;
    Eigen::Map<const Jacobian> J(jacobian);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> dx(delta);
    Eigen::Matrix<double, 6, 6> A = J * J.transpose();
    A.diagonal().array() += damping * damping;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(output);
    dq = J.transpose() * A.ldlt().solve(dx);
}
