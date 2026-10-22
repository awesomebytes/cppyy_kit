import numpy as np
from _native import sum_sq as native_sum


def sum_sq(values):
    return native_sum(np.ascontiguousarray(values, dtype=np.float64))
