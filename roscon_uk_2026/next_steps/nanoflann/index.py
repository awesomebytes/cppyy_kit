"""Checkout experiment: copy positions into a retained native nanoflann index."""
from __future__ import annotations

import hashlib
import operator
from pathlib import Path
from time import perf_counter

HERE = Path(__file__).resolve().parent
# Pixi activates the current checkout's require/cache implementation.
_start = perf_counter()
import cppyy_kit
import cppyy
import numpy as np
IMPORT_SECONDS = perf_counter() - _start

VERSION = "1.6.3"
HEADER_SHA256 = "e47f12ae2fc339cd2eb9ca148af837bd1025ff7e301ecf1860675ff1c73dd071"
URL = f"https://raw.githubusercontent.com/jlblancoc/nanoflann/v{VERSION}/include/nanoflann.hpp"
_start = perf_counter()
LIBRARY = cppyy_kit.require(
    "nanoflann-1.6.3", "nanoflann.hpp", url=URL, sha256=HEADER_SHA256,
    search_paths=("/usr/include", "/usr/local/include"),
    cache_dir=str(HERE / "build/vendor"))
HEADER_PATH = Path(LIBRARY["include_dir"]) / "nanoflann.hpp"
ACTUAL_HEADER_SHA256 = hashlib.sha256(HEADER_PATH.read_bytes()).hexdigest()
# require verifies downloads and cached content. Check the resolved header too
# so installed headers must also match this experiment's pinned revision.
if ACTUAL_HEADER_SHA256 != HEADER_SHA256:
    raise RuntimeError(f"Expected nanoflann {VERSION} header {HEADER_SHA256}; "
                       f"found {ACTUAL_HEADER_SHA256} at {HEADER_PATH}")
HEADER_SECONDS = perf_counter() - _start
_start = perf_counter()
_code = (HERE / "native.cpp").read_text()
_decls = (HERE / "native.hpp").read_text()
_options = dict(name="recorded_nn", include_paths=(str(HERE), LIBRARY["include_dir"]),
                directory=str(HERE / "build/cache"))
# Prebuild exposes only declarations to Cling, including on the first run.
# Heavy templates remain compiled by the pinned C++ compiler.
cppyy_kit.prebuild(_code, decls=_decls, **_options)
PREBUILD_SECONDS = perf_counter() - _start
_start = perf_counter()
COMPILE = cppyy_kit.cppdef_cached(_code, decls=_decls, **_options)
COMPILE_SECONDS = perf_counter() - _start


def positions(value):
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("positions must have shape (n, 3)")
    if not np.isfinite(array).all() or np.any(np.abs(array) > 1e150):
        raise ValueError("coordinates must be finite and abs <= 1e150")
    return np.require(array, requirements=["C", "A"])


def frame_ids(value, n):
    array = np.asarray(value)
    if array.shape != (n,) or (array.size and array.dtype.kind not in "iu"):
        raise ValueError("frames must be an integer array of shape (n,)")
    if np.any(array < 0) or np.any(array > 2**62):
        raise ValueError("frame must be in [0, 2**62]")
    return np.require(array, dtype=np.int64, requirements=["C", "A"])


def buffer(array):
    # cppyy rejects a zero-length NumPy buffer. Native n/m=0 guarantees that
    # the one-element sentinel is never dereferenced.
    return array.ravel() if array.size else np.zeros(1, dtype=array.dtype)


class Index:
    """Owned immutable coordinates, frames and confidence, with explicit close."""

    def __init__(self, xyz, frames, confidence):
        start = perf_counter()
        xyz = positions(xyz)
        frames = frame_ids(frames, len(xyz))
        confidence = np.require(confidence, dtype=np.float64, requirements=["C", "A"])
        if confidence.shape != (len(xyz),):
            raise ValueError("confidence must have shape (n,)")
        if not np.isfinite(confidence).all() or np.any((confidence < 0) | (confidence > 1)):
            raise ValueError("confidence must be finite in [0, 1]")
        self.input_conversion_seconds = perf_counter() - start
        start = perf_counter()
        self._native = cppyy.gbl.recorded_nn.Index(buffer(xyz), buffer(frames),
                                                  buffer(confidence), len(xyz))
        self.constructor_seconds = perf_counter() - start
        self.build_seconds = self._native.build_seconds()
        self.size = len(xyz)

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def query(self, xyz, frames, k=4, frame_gap=-1, min_confidence=0.0,
              max_distance=np.inf):
        start = perf_counter()
        if isinstance(k, bool):
            raise ValueError("k must be an integer in [1, 4096]")
        k = operator.index(k)
        frame_gap = operator.index(frame_gap)
        if not 1 <= k <= 4096:
            raise ValueError("k must be in [1, 4096]")
        if not -1 <= frame_gap <= 2**62:
            raise ValueError("frame_gap must be in [-1, 2**62]")
        xyz = positions(xyz)
        frames = frame_ids(frames, len(xyz))
        m = len(xyz)
        ids = np.empty((m, k), dtype=np.int64)
        d2 = np.empty((m, k), dtype=np.float64)
        counts = np.empty(m, dtype=np.int64)
        centroids = np.empty((m, 3), dtype=np.float64)
        conversion_seconds = perf_counter() - start
        start = perf_counter()
        query_seconds = self._native.query(
            buffer(xyz), buffer(frames), m, k, frame_gap, min_confidence, max_distance,
            buffer(ids), buffer(d2), buffer(counts), buffer(centroids))
        call_seconds = perf_counter() - start
        # Native writes directly to caller-owned NumPy arrays. No output copy.
        return dict(ids=ids, distances_squared=d2, counts=counts,
                    centroids=centroids, native_seconds=query_seconds,
                    call_seconds=call_seconds, conversion_seconds=conversion_seconds)
