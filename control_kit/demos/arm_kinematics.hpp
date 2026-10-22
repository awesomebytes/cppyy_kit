#pragma once
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <vector>

namespace demo_arm_kinematics {
// The chain is ordered from base to tip. Origins contain xyz and fixed-axis rpy.
void kinematics(int segments, int joints, const int* types, const int* indices,
                const double* origins, const double* axes, const double* q,
                double* pose, double* jacobian) {
  using namespace Eigen;
  Isometry3d tip = Isometry3d::Identity();
  std::vector<Vector3d> points(joints), directions(joints);
  std::vector<int> kinds(joints);
  for (int s = 0; s < segments; ++s) {
    Isometry3d origin = Isometry3d::Identity();
    origin.translation() = Map<const Vector3d>(origins + 6*s);
    origin.linear() = (AngleAxisd(origins[6*s+5], Vector3d::UnitZ()) *
                       AngleAxisd(origins[6*s+4], Vector3d::UnitY()) *
                       AngleAxisd(origins[6*s+3], Vector3d::UnitX())).toRotationMatrix();
    tip = tip * origin;
    int j = indices[s];
    if (j < 0) continue;
    Vector3d axis = Map<const Vector3d>(axes + 3*s);
    points[j] = tip.translation();
    directions[j] = tip.linear() * axis;
    kinds[j] = types[s];
    if (types[s] == 1) tip.rotate(AngleAxisd(q[j], axis));
    else tip.translate(q[j] * axis);
  }
  Map<Matrix<double,4,4,RowMajor>> output_pose(pose);
  output_pose = tip.matrix();
  Map<Matrix<double,6,Dynamic,RowMajor>> j(jacobian, 6, joints);
  for (int k = 0; k < joints; ++k) {
    j.block<3,1>(0,k) = kinds[k] == 1
        ? Vector3d(directions[k].cross(tip.translation() - points[k])) : directions[k];
    j.block<3,1>(3,k) = kinds[k] == 1 ? directions[k] : Vector3d::Zero();
  }
}

// One native call evaluates FK, the analytic geometric Jacobian and damped IK.
void step(int segments, int joints, const int* types, const int* indices,
          const double* origins, const double* axes, const double* q,
          const double* target, const double* lower, const double* upper,
          double dt, double gain, double damping, double speed, double* command) {
  using namespace Eigen;
  Matrix<double,4,4,RowMajor> pose;
  Matrix<double,6,Dynamic,RowMajor> j(6,joints);
  kinematics(segments,joints,types,indices,origins,axes,q,pose.data(),j.data());
  Map<const Matrix<double,4,4,RowMajor>> goal(target);
  Matrix<double,6,1> error;
  error.head<3>() = goal.block<3,1>(0,3) - pose.block<3,1>(0,3);
  error.tail<3>().setZero();
  for (int k=0;k<3;++k)
    error.tail<3>() += 0.5 * pose.block<3,1>(0,k).cross(goal.block<3,1>(0,k));
  Matrix<double,6,6> h = j*j.transpose();
  h.diagonal().array() += damping*damping;
  VectorXd velocity = j.transpose()*h.ldlt().solve(gain*error);
  // Scale the whole velocity vector to preserve its direction at the speed bound.
  double scale = std::max(1.0, velocity.cwiseAbs().maxCoeff()/speed);
  for (int k=0;k<joints;++k)
    command[k] = std::clamp(q[k]+dt*velocity[k]/scale,lower[k],upper[k]);
}
}
