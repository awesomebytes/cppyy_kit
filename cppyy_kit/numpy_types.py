"""NumPy typing aliases used by cppyy_kit."""
from typing import Annotated, TypeAlias, TypeVar

import numpy as np
from numpy import (bool_, complex64, complex128, float32, float64, int8, int16,
                   int32, int64, intc, intp, uint8, uint16, uint32, uint64,
                   uintp)
from numpy.typing import ArrayLike, DTypeLike, NDArray

_T = TypeVar("_T", bound=np.generic)
_CONST_NDARRAY = object()

ConstNDArray: TypeAlias = Annotated[NDArray[_T], _CONST_NDARRAY]

__all__ = [
    "ArrayLike", "ConstNDArray", "DTypeLike", "NDArray", "bool_", "complex64",
    "complex128", "float32", "float64", "int8", "int16", "int32", "int64",
    "intc", "intp", "uint8", "uint16", "uint32", "uint64", "uintp",
]
