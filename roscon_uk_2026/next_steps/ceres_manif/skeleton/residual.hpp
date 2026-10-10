#pragma once
#include <ceres/ceres.h>
#include <manif/SE3.h>
#include <manif/ceres/ceres.h>
namespace calibration {
struct PointResidual {
  double source[3], target[3];
  template <typename T> bool operator()(const T* x, T* residual) const {
    // Implement prediction minus measurement using manif group action.
    // x: target-frame translation (m), then SO(3) rotation vector (rad).
    return false;
  }
};
}
