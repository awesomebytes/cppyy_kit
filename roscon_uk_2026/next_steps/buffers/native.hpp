#pragma once
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace buffer_demo {
using Points = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Rotation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using Input = Eigen::Map<const Points, Eigen::Unaligned>;
using Output = Eigen::Map<Points, Eigen::Unaligned>;

inline void transform(const double* points, std::size_t size,
                      const double* rotation, std::size_t rotation_size,
                      const double* translation, std::size_t translation_size,
                      double* output, std::size_t output_size) {
    if (size % 3 || rotation_size != 9 || translation_size != 3 || output_size != size)
        throw std::invalid_argument("transform buffer sizes");
    if (!size) return;
    Input source(points, size / 3, 3);
    Eigen::Map<const Rotation, Eigen::Unaligned> r(rotation);
    Eigen::Map<const Eigen::Vector3d, Eigen::Unaligned> t(translation);
    Output target(output, size / 3, 3);
    target.noalias() = source * r.transpose();
    target.rowwise() += t.transpose();
}

inline std::size_t filter(const double* points, std::size_t size,
                          double radius_squared, double* output,
                          std::size_t output_size) {
    if (size % 3 || output_size != size)
        throw std::invalid_argument("filter buffer sizes");
    if (!size) return 0;
    Input source(points, size / 3, 3);
    Output target(output, size / 3, 3);
    std::size_t kept = 0;
    for (Eigen::Index i = 0; i < source.rows(); ++i) {
        // Preserve input order; reject nonfinite transformed coordinates.
        if (source.row(i).allFinite() && source.row(i).squaredNorm() <= radius_squared)
            target.row(kept++) = source.row(i);
    }
    return kept;
}

inline double energy(const double* points, std::size_t size) {
    if (size % 3) throw std::invalid_argument("energy buffer size");
    if (!size) return 0.0;
    Input source(points, size / 3, 3);
    return source.squaredNorm();
}

inline int eigen_version() {
    return EIGEN_MAJOR_VERSION * 10000 + EIGEN_MINOR_VERSION * 100 + EIGEN_PATCH_VERSION;
}
inline long cpp_standard() { return __cplusplus; }
}
