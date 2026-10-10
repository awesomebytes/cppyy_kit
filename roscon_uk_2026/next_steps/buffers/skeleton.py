"""Copy to candidate.py, then implement only the two missing C++ bodies."""
import numpy as np
from buffers import cpp, ConstNDArray, NDArray, load_native

load_native()


@cpp(cached=False, std="c++17")
def transform_kernel(points: ConstNDArray[np.float64], rotation: ConstNDArray[np.float64],
                     translation: ConstNDArray[np.float64], output: NDArray[np.float64]):
    """
    // TODO: Validate element counts. Use row-major Eigen::Map views.
    // For each point row, output = point * rotation.transpose() + translation.
    throw std::logic_error("implement transform_kernel");
    """


@cpp(cached=False, std="c++17")
def filter_kernel(points: ConstNDArray[np.float64], radius_squared: float,
                  output: NDArray[np.float64]) -> np.uintp:
    """
    // TODO: Validate element counts. Keep finite rows within radius_squared.
    // Preserve order, write retained rows to output, and return the row count.
    throw std::logic_error("implement filter_kernel");
    """
