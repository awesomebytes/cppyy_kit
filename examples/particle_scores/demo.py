"""Parallel range-only localisation scores using Eigen and OpenMP."""
import os
from pathlib import Path

import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import NDArray

EIGEN_INCLUDE = str(Path(os.environ['CONDA_PREFIX']) / 'include/eigen3')


@cpp(headers=['Eigen/Core', 'omp.h', 'stdexcept'],
     include_paths=[EIGEN_INCLUDE], link_args=['-fopenmp'], nogil=True,
     preamble='#ifndef _OPENMP\n#error OpenMP compilation is required\n#endif')
def score_particles(positions: NDArray[np.float64],
                    landmarks: NDArray[np.float64],
                    ranges: NDArray[np.float64],
                    scores: NDArray[np.float64], threads: int) -> None:
    """
    if (positions_size != 2 * scores_size || landmarks_size != 2 * ranges_size
        || threads < 1)
        throw std::invalid_argument("Expected XY pairs, one score per particle, and threads >= 1");
    #pragma omp parallel for num_threads(threads) schedule(static)
    for (long i = 0; i < static_cast<long>(scores_size); ++i) {
        Eigen::Map<const Eigen::Vector2d> position(positions + 2 * i);
        double error = 0.0;
        for (size_t j = 0; j < ranges_size; ++j) {
            Eigen::Map<const Eigen::Vector2d> landmark(landmarks + 2 * j);
            double residual = (position - landmark).norm() - ranges[j];
            error += residual * residual;
        }
        scores[i] = -error;
    }
    """


def numpy_scores(positions, landmarks, ranges):
    """Broadcasting reference; use separate X/Y arrays to limit intermediates."""
    dx = positions[:, 0, None] - landmarks[None, :, 0]
    dy = positions[:, 1, None] - landmarks[None, :, 1]
    np.square(dx, out=dx)
    np.square(dy, out=dy)
    dx += dy
    np.sqrt(dx, out=dx)
    dx -= ranges
    np.square(dx, out=dx)
    return -dx.sum(axis=1)


def numpy_streamed_scores(positions, landmarks, ranges):
    """Process one landmark at a time, using O(particles) temporary storage."""
    scores = np.zeros(len(positions))
    for landmark, distance in zip(landmarks, ranges):
        dx = positions[:, 0] - landmark[0]
        dy = positions[:, 1] - landmark[1]
        np.square(dx, out=dx)
        np.square(dy, out=dy)
        dx += dy
        np.sqrt(dx, out=dx)
        dx -= distance
        np.square(dx, out=dx)
        scores -= dx
    return scores


def inputs(count=20000, observations=32):
    rng = np.random.default_rng(7)
    positions = rng.uniform(-10, 10, size=(count, 2))
    landmarks = rng.uniform(-10, 10, size=(observations, 2))
    true_position = np.array([1.0, -2.0])
    ranges = np.linalg.norm(landmarks - true_position, axis=1)
    return positions, landmarks, ranges, np.empty(count)


if __name__ == '__main__':
    positions, landmarks, ranges, scores = inputs()
    score_particles(positions, landmarks, ranges, scores, 4)
    np.testing.assert_allclose(scores, numpy_scores(positions, landmarks, ranges),
                               rtol=1e-12, atol=1e-10)
    print('Best candidate XY:', positions[scores.argmax()])
