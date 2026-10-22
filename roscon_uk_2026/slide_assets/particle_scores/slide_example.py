import sys
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import NDArray

Array = NDArray[np.float64]

@cpp(headers=['Eigen/Core', 'stdexcept'],
     include_paths=[f'{sys.prefix}/include/eigen3'],
     link_args=['-fopenmp'], nogil=True)
def score(positions: Array, landmarks: Array, ranges: Array,
          scores: Array, threads: int = 4) -> None:
    """
    using Vec = Eigen::Vector2d;
    if (positions_size != 2*scores_size || landmarks_size != 2*ranges_size
        || threads < 1)
        throw std::invalid_argument("Invalid dimensions or thread count");
    #pragma omp parallel for num_threads(threads) schedule(static)
    for (long i = 0; i < static_cast<long>(scores_size); ++i) {
        Eigen::Map<const Vec> position(positions + 2*i);
        double error = 0.0;
        for (size_t j = 0; j < ranges_size; ++j) {
            Eigen::Map<const Vec> landmark(landmarks + 2*j);
            double residual = (position - landmark).norm() - ranges[j];
            error += residual * residual;
        }
        scores[i] = -error;
    }
    """
