#pragma once
#include <ceres/ceres.h>
#include <manif/SE3.h>
#include <manif/ceres/ceres.h>
namespace calibration {
// x = translation in target frame (m), then rotation vector (rad).
// T_target_source acts on a source-frame point to predict a target-frame point.
struct PointResidual {
  double source[3], target[3];
  template <typename T> bool operator()(const T* x, T* residual) const {
    Eigen::Matrix<T,3,1> rotation_vector(x[3], x[4], x[5]);
    Eigen::Matrix<T,3,1> translation(x[0], x[1], x[2]);
    Eigen::Matrix<T,3,1> point{T(source[0]), T(source[1]), T(source[2])};
    manif::SE3<T> transform(translation, manif::SO3Tangent<T>(rotation_vector).exp());
    const auto prediction = transform.act(point);
    for (int j=0; j<3; ++j) residual[j] = prediction[j] - T(target[j]);
    return true;
  }
};
}
