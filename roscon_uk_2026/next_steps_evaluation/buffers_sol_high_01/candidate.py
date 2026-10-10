"""Copy to candidate.py, then implement only the two missing C++ bodies."""
import numpy as np
from buffers import cpp, ConstNDArray, NDArray, load_native

load_native()


@cpp(cached=False, std="c++17")
def transform_kernel(points: ConstNDArray[np.float64], rotation: ConstNDArray[np.float64],
                     translation: ConstNDArray[np.float64], output: NDArray[np.float64]):
    """
    if (points_size % 3 != 0 || output_size != points_size ||
        rotation_size != 9 || translation_size != 3) {
        throw std::invalid_argument("invalid transform buffer sizes");
    }
    if (points_size == 0) {
        return;
    }
    using PointMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
    using RotationMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    const Eigen::Index rows = static_cast<Eigen::Index>(points_size / 3);
    Eigen::Map<const PointMatrix, Eigen::Unaligned> source(points, rows, 3);
    Eigen::Map<PointMatrix, Eigen::Unaligned> target(output, rows, 3);
    Eigen::Map<const RotationMatrix, Eigen::Unaligned> rigid_rotation(rotation);
    Eigen::Map<const Eigen::Vector3d, Eigen::Unaligned> offset(translation);
    for (Eigen::Index row = 0; row < rows; ++row) {
        const Eigen::RowVector3d transformed =
            source.row(row) * rigid_rotation.transpose() + offset.transpose();
        target.row(row) = transformed;
    }
    """


@cpp(cached=False, std="c++17")
def filter_kernel(points: ConstNDArray[np.float64], radius_squared: float,
                  output: NDArray[np.float64]) -> np.uintp:
    """
    if (points_size % 3 != 0 || output_size != points_size) {
        throw std::invalid_argument("invalid filter buffer sizes");
    }
    if (points_size == 0) {
        return 0;
    }
    using PointMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
    const Eigen::Index rows = static_cast<Eigen::Index>(points_size / 3);
    Eigen::Map<const PointMatrix, Eigen::Unaligned> source(points, rows, 3);
    Eigen::Map<PointMatrix, Eigen::Unaligned> target(output, rows, 3);
    std::size_t retained = 0;
    for (Eigen::Index row = 0; row < rows; ++row) {
        const Eigen::RowVector3d point = source.row(row);
        if (std::isfinite(point[0]) && std::isfinite(point[1]) &&
            std::isfinite(point[2]) && point.squaredNorm() <= radius_squared) {
            target.row(static_cast<Eigen::Index>(retained)) = point;
            ++retained;
        }
    }
    return retained;
    """
