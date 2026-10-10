"""Task-specific leases for synchronous NumPy/Eigen point processing."""
from pathlib import Path
import sys
import time

import numpy as np

# This experiment explicitly imports the core package from this checkout.
CHECKOUT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(CHECKOUT))
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray, NDArray
from cppyy_kit.require import require

_loaded = False


def load_native():
    """Load Eigen declarations once; return the elapsed declaration-load time."""
    global _loaded
    if _loaded:
        return 0.0
    import cppyy
    start = time.perf_counter()
    eigen = require("Eigen", "Eigen/Core", search_paths=[str(Path(sys.prefix) / "include/eigen3")])
    cppyy.add_include_path(eigen["include_dir"])
    cppyy.include(str(Path(__file__).with_name("native.hpp")))
    _loaded = True
    return time.perf_counter() - start


# Header declarations exist in Cling. These wrappers use the existing typed
# marshaling path; cached=False avoids compiling a separate translation unit
# without those declarations.
@cpp(cached=False, std="c++17")
def transform_kernel(points: ConstNDArray[np.float64], rotation: ConstNDArray[np.float64],
                     translation: ConstNDArray[np.float64], output: NDArray[np.float64]):
    """
    buffer_demo::transform(points, points_size, rotation, rotation_size,
                           translation, translation_size, output, output_size);
    """


@cpp(cached=False, std="c++17")
def filter_kernel(points: ConstNDArray[np.float64], radius_squared: float,
                  output: NDArray[np.float64]) -> np.uintp:
    """
    return buffer_demo::filter(points, points_size, radius_squared, output, output_size);
    """


@cpp(cached=False, std="c++17")
def energy_kernel(points: ConstNDArray[np.float64]) -> float:
    """
    return buffer_demo::energy(points, points_size);
    """


@cpp(cached=False, std="c++17")
def pointer_kernel(points: ConstNDArray[np.float64]) -> np.uintp:
    """
    return reinterpret_cast<std::uintptr_t>(points);
    """


class BorrowedPoints:
    """Keep a point array and buffer export alive until close.

    No C++ pointer survives a call. Callers must not mutate or resize underlying
    storage during processing, including through aliases or refcheck=False.
    """
    def __init__(self, points, *, copy=False):
        if not isinstance(points, np.ndarray):
            raise TypeError("points must be a NumPy ndarray")
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        # Conversion is opt-in and restricted to real floating-point inputs.
        if points.dtype.kind != "f" or points.dtype.itemsize not in (4, 8):
            raise TypeError("points must use float32 or float64; integer/object data is rejected")
        compatible = points.dtype == np.dtype(np.float64) and points.dtype.isnative
        compatible = compatible and points.flags.c_contiguous and points.flags.aligned
        if not compatible and not copy:
            raise TypeError("points require native float64, aligned C-contiguous storage; pass copy=True")
        self.copied = not compatible
        self._array = (np.array(points, dtype=np.float64, order="C", copy=True)
                       if self.copied else points)
        if not np.isfinite(self._array).all():
            raise ValueError("points must contain finite coordinates")
        self._export = memoryview(self._array)
        self._stamp = self._metadata()
        self._closed = False

    def _metadata(self):
        a = self._array
        return (int(a.ctypes.data), a.shape, a.strides, a.dtype.str, a.flags.c_contiguous,
                a.flags.aligned, a.nbytes)

    @property
    def array(self):
        self.validate()
        return self._array

    def validate(self):
        if self._closed:
            raise RuntimeError("point lease is closed")
        if self._metadata() != self._stamp:
            raise RuntimeError("point storage or layout changed; close and create a new lease")

    def close(self):
        if not self._closed:
            self._export.release()
            self._array = None
            self._closed = True

    def __enter__(self):
        self.validate()
        return self

    def __exit__(self, *exc):
        self.close()


def rigid_parameters(rotation, translation, radius):
    r = np.asarray(rotation)
    t = np.asarray(translation)
    if r.shape != (3, 3) or t.shape != (3,):
        raise ValueError("rotation must be (3, 3) and translation (3,)")
    if r.dtype != np.dtype(np.float64) or t.dtype != np.dtype(np.float64):
        raise TypeError("rotation and translation must use native float64")
    if not r.flags.c_contiguous or not t.flags.c_contiguous or not r.flags.aligned or not t.flags.aligned:
        raise TypeError("rotation and translation must be aligned and C-contiguous")
    if not np.isfinite(r).all() or not np.isfinite(t).all():
        raise ValueError("rigid parameters must be finite")
    if not np.allclose(r @ r.T, np.eye(3), rtol=0, atol=1e-12) or not np.isclose(np.linalg.det(r), 1, rtol=0, atol=1e-12):
        raise ValueError("rotation must be orthogonal with determinant +1")
    if not np.isfinite(radius) or radius < 0 or radius > np.sqrt(np.finfo(np.float64).max):
        raise ValueError("radius must be finite, nonnegative and squareable")
    return r, t, float(radius) ** 2


class PipelineResult:
    def __init__(self, transformed, storage, count, energy):
        self.transformed = transformed
        self.storage = storage
        self.points = storage[:count]
        self.points.flags.writeable = False
        self.energy = energy


def process(points, rotation, translation, radius, *, copy=False):
    """Transform points, retain those inside the sphere, then sum squared ranges.

    Distances use the same unit as the input coordinates. Energy has squared
    distance units. The sphere is centered at the target-frame origin.
    """
    load_native()
    owned_lease = not isinstance(points, BorrowedPoints)
    lease = BorrowedPoints(points, copy=copy) if owned_lease else points
    try:
        source = lease.array
        r, t, radius_squared = rigid_parameters(rotation, translation, radius)
        # Both allocations belong to NumPy. Native calls never retain pointers.
        transformed = np.empty(source.shape, dtype=np.float64)
        storage = np.empty(source.shape, dtype=np.float64)
        transform_kernel(source, r, t, transformed)
        count = int(filter_kernel(transformed, radius_squared, storage))
        selected = storage[:count]
        selected.flags.writeable = False
        energy = energy_kernel(selected)
        return PipelineResult(transformed, storage, count, energy)
    finally:
        if owned_lease:
            lease.close()


def dlpack_points(producer):
    """Import CPU DLPack without a copy, then apply the same borrowing policy."""
    if producer.__dlpack_device__()[0] != 1:
        raise ValueError("this experiment supports CPU DLPack only")
    imported = np.from_dlpack(producer, copy=False)
    return BorrowedPoints(imported)
