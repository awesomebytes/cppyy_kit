#include "adapter.hpp"
#include "residual.hpp"
#include <cmath>
#include <stdexcept>
namespace calibration {
using Cost = ceres::AutoDiffCostFunction<PointResidual,3,6>;
PointResidual* make_residual(const double* p, const double* q) {
  auto* r = new PointResidual;
  for(int j=0;j<3;++j) { r->source[j]=p[j]; r->target[j]=q[j]; }
  return r;
}
Report solve(const double* source, const double* target, int count, double* x,
             double scale, int max_iterations, double tolerance) {
  if(count<3 || scale<0 || max_iterations<0 || !std::isfinite(scale) ||
     !std::isfinite(tolerance) || tolerance<=0)
    throw std::invalid_argument("invalid solve options or point count");
  for(int j=0;j<6;++j) if(!std::isfinite(x[j])) throw std::invalid_argument("nonfinite initial transform");
  Eigen::MatrixXd centered(count,3);
  for(int i=0;i<count;++i) for(int j=0;j<3;++j) {
    if(!std::isfinite(source[3*i+j]) || !std::isfinite(target[3*i+j]))
      throw std::invalid_argument("nonfinite observation");
    centered(i,j)=source[3*i+j];
  }
  centered.rowwise()-=centered.colwise().mean().eval();
  auto singular = centered.jacobiSvd().singularValues();
  if(singular[0]<1e-9 || singular[1]<singular[0]*1e-6)
    throw std::invalid_argument("degenerate source points: rotation around line is unobservable");
  ceres::Problem problem;
  for(int i=0;i<count;++i) problem.AddResidualBlock(
    new Cost(make_residual(source+3*i,target+3*i)),
    scale>0 ? new ceres::HuberLoss(scale) : nullptr, x);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = 1;
  options.max_num_iterations = max_iterations;
  options.function_tolerance = tolerance;
  options.gradient_tolerance = tolerance;
  options.parameter_tolerance = tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  return {int(summary.termination_type),int(summary.iterations.size()),
          summary.IsSolutionUsable(),summary.termination_type==ceres::CONVERGENCE,
          summary.initial_cost,summary.final_cost,summary.total_time_in_seconds};
}
void evaluate(const double* p, const double* q, const double* x,
              double* residual, double* jacobian) {
  Cost cost(make_residual(p,q));
  const double* parameters[] = {x};
  double* jacobians[] = {jacobian};
  if(!cost.Evaluate(parameters,residual,jacobians)) throw std::runtime_error("residual evaluation failed");
}
void group_action(const double* x, const double* p, const double* delta,
                  double* output, double* jacobian) {
  Eigen::Vector3d translation(x[0],x[1],x[2]), rotation(x[3],x[4],x[5]);
  manif::SE3d transform(translation,manif::SO3Tangentd(rotation).exp());
  Eigen::Map<const Eigen::Vector3d> point(p);
  Eigen::Matrix<double,3,6> J;
  transform.act(point,J);
  Eigen::Map<const Eigen::Matrix<double,6,1>> d(delta);
  Eigen::Map<Eigen::Vector3d> result(output);
  result = transform.rplus(manif::SE3Tangentd(d)).act(point);
  for(int i=0;i<3;++i) for(int j=0;j<6;++j) jacobian[6*i+j]=J(i,j);
}
void toolchain(int* output) {
  output[0]=__cplusplus; output[1]=EIGEN_MAX_ALIGN_BYTES;
  output[2]=_GLIBCXX_USE_CXX11_ABI;
  output[3]=EIGEN_WORLD_VERSION; output[4]=EIGEN_MAJOR_VERSION; output[5]=EIGEN_MINOR_VERSION;
}
}
