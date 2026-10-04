"""Public NumPy annotations for read-only @cpp input buffers."""
from typing import Annotated, TypeAlias, TypeVar

import numpy as np
from numpy.typing import NDArray

_T = TypeVar("_T", bound=np.generic)
_CONST_NDARRAY = object()

ConstNDArray: TypeAlias = Annotated[NDArray[_T], _CONST_NDARRAY]

__all__ = ["ConstNDArray"]
