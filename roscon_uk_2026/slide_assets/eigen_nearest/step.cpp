#include <Eigen/Core>
#include <limits>

int nearest_point(const double* xyz, size_t count) {
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < count; ++i) {
        Eigen::Map<const Eigen::Vector3d> point(xyz + 3 * i);
        double d2 = point.squaredNorm();
        if (d2 < best_d2) {
            best_d2 = d2;
            best = static_cast<int>(i);
        }
    }
    return best;
}
