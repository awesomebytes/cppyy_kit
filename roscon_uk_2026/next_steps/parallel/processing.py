"""Freeze a batch before calling GIL-free native kernels."""
import ctypes
import math
import threading
import time

import cppyy
import numpy as np
from cppyy_kit import cpp, pydantic_structs as pyd

from build import BUILD, HERE, build
from model import Detection, validate

_LOADED = False
_LIBRARY = None
_FUNCTIONS = {}


def load():
    global _LOADED, _LIBRARY
    metadata = build()
    if not _LOADED:
        cppyy.load_library(str(BUILD / "libdetection.so"))
        cppyy.add_include_path(str(HERE))
        cppyy.include("native.h")
        _LIBRARY = ctypes.CDLL(str(BUILD / "libdetection.so"))
        for name in ("detection_aos", "detection_columns"):
            _FUNCTIONS[name] = ctypes.cast(getattr(_LIBRARY, name), ctypes.c_void_p).value
        _LOADED = True
    return metadata


@cpp(nogil=True)
def _aos(function: "std::uintptr_t", records: "std::uintptr_t", n: int, frames: int, radius2: float, threshold: float,
         parallel: int, workers: int, flags: "unsigned char*", counts: "std::uint64_t*",
         peak: "int*") -> None:
    """using Fn = void (*)(std::uintptr_t, std::size_t, std::size_t, double, double, int, int,
                           unsigned char*, std::uint64_t*, int*);
    reinterpret_cast<Fn>(function)(records, n, frames, radius2, threshold, parallel, workers, flags, counts, peak);
    """


@cpp(nogil=True)
def _columns(function: "std::uintptr_t", x: "const double*", y: "const double*", z: "const double*",
             confidence: "const double*", frame: "const std::int64_t*",
             n: int, frames: int, radius2: float, threshold: float, simd: int,
             flags: "unsigned char*", counts: "std::uint64_t*") -> None:
    """using Fn = void (*)(const double*, const double*, const double*, const double*, const std::int64_t*,
                           std::size_t, std::size_t, double, double, int, unsigned char*, std::uint64_t*);
    reinterpret_cast<Fn>(function)(x, y, z, confidence, frame, n, frames, radius2, threshold, simd, flags, counts);
    """


class Batch:
    """Own native arrays and the prototype's native vector for all calls.

    No public input mutation or resize API is exposed. A lock serializes calls
    on this batch while the GIL is released. Result arrays belong to each call.
    """
    def __init__(self, records, frames=64, layout="both"):
        if layout not in {"both", "aos", "columns"}:
            raise ValueError("layout must be both, aos, or columns")
        load()
        self.layout = layout
        self._frames = frames
        self._lock = threading.Lock()
        start = time.perf_counter()
        models = validate(records, frames)
        self.costs = {"validation_ms": (time.perf_counter() - start)*1000}
        self._n = len(models)
        start = time.perf_counter()
        self._columns = {field: np.fromiter((getattr(m, field) for m in models),
                                           dtype=np.int64 if field == "frame" else np.float64,
                                           count=self.n)
                         for field in Detection.model_fields}
        self.costs["columns_ms"] = (time.perf_counter() - start)*1000
        start = time.perf_counter()
        self.costs["aos_fill_ms"] = 0.
        self.native_bytes = 0
        if layout != "columns":
            spec = pyd.cpp_struct(Detection)
            if self.n:
                self._records = pyd.cpp_vector_columnar(Detection, self._columns)
            else:
                # The prototype's ctypes column helper cannot form an empty view at
                # a nonzero member offset. Use its general vector path for zero rows.
                self._records = pyd.cpp_vector(Detection, [])
            self._address = int(spec._helper("_vec_data")(self._records))
            self.costs["aos_fill_ms"] = (time.perf_counter() - start)*1000
            self.native_bytes += self.n * int(spec._helper("_sizeof")())
        # cpp_vector_columnar pins its source columns. Retention is part of
        # the prototype contract, so columns remain live with an AoS vector.
        self.native_bytes += sum(a.nbytes for a in self._columns.values())
        for array in self._columns.values():
            array.flags.writeable = False

    @property
    def n(self):
        return self._n

    @property
    def frames(self):
        return self._frames

    def run(self, method="serial_aos", workers=1, radius=5., threshold=.7, instrument=False):
        if method not in {"serial_aos", "tbb", "serial_columns", "xsimd"}:
            raise ValueError("unknown method")
        if self.layout == "columns" and method in {"serial_aos", "tbb"}:
            raise ValueError("this batch has no array-of-structs storage")
        if type(workers) is not int or not 1 <= workers <= 32:
            raise ValueError("workers must be an integer in [1,32]")
        if type(radius) not in (int, float) or not math.isfinite(radius) or not 0 <= radius <= 1000:
            raise ValueError("radius must be finite and in [0,1000] metres")
        if type(threshold) not in (int, float) or not math.isfinite(threshold) or not 0 <= threshold <= 1:
            raise ValueError("threshold must be finite and in [0,1]")
        flags = np.empty(self.n, dtype=np.uint8)
        counts = np.empty(self.frames, dtype=np.uint64)
        peak = np.zeros(1, dtype=np.int32)
        with self._lock:
            if method in {"serial_aos", "tbb"}:
                _aos(_FUNCTIONS["detection_aos"], self._address, self.n, self.frames, radius*radius, threshold,
                     int(method == "tbb"), workers, flags, counts, peak if instrument else 0)
            else:
                _columns(_FUNCTIONS["detection_columns"], *(self._columns[f] for f in Detection.model_fields),
                         self.n, self.frames, radius*radius, threshold, int(method == "xsimd"), flags, counts)
        return flags, counts, int(peak[0])


def lanes():
    load()
    return int(cppyy.gbl.detection_lanes())
