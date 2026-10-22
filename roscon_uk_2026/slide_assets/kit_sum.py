"""Expose and cache the reduction with cppyy_kit."""
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import NDArray
from measure import run


@cpp
def sum_sq(x: NDArray[np.float64]) -> float:
    """
    double total = 0;
    for (size_t i = 0; i < x_size; ++i) total += x[i] * x[i];
    return total;
    """


if __name__ == '__main__':
    run(sum_sq)
