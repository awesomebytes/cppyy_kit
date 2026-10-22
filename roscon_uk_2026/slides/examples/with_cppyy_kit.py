import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import NDArray

@cpp
def sum_sq(x: NDArray[np.float64]) -> float:
    """
    double total = 0;
    for (size_t i = 0; i < x_size; ++i)
        total += x[i] * x[i];
    return total;
    """

x = np.array([1., 2., 3.])
print(sum_sq(x))
