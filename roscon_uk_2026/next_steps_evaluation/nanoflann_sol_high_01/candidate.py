"""Retained native nanoflann index with filtered exact batch queries."""
import hashlib
import operator
from pathlib import Path
import sys
import time

import numpy as np

_BUILD = Path(__file__).resolve().parent / 'build'
_CHECKSUM = 'e47f12ae2fc339cd2eb9ca148af837bd1025ff7e301ecf1860675ff1c73dd071'
_NATIVE = None
_SETUP_TIMINGS = {}


def _native_class():
    global _NATIVE
    if _NATIVE is not None:
        return _NATIVE
    import cppyy
    from cppyy_kit import require, prebuild, cppdef_cached

    start = time.perf_counter()
    dependency = require(
        'nanoflann', 'nanoflann.hpp',
        search_paths=(str(Path(sys.prefix) / 'include'), '/usr/include', '/usr/local/include'),
        url='https://raw.githubusercontent.com/jlblancoc/nanoflann/v1.6.3/include/nanoflann.hpp',
        sha256=_CHECKSUM, cache_dir=str(_BUILD / 'vendor'))
    header = Path(dependency['include_dir']) / 'nanoflann.hpp'
    digest = hashlib.sha256(header.read_bytes()).hexdigest()
    if digest != _CHECKSUM:
        raise RuntimeError(f'nanoflann v1.6.3 checksum mismatch at {header}: '
                           f'expected {_CHECKSUM}, got {digest}')
    source_dir = _BUILD / 'agent_native'
    code = (source_dir / 'retained.cpp').read_text()
    decls = (source_dir / 'retained.hpp').read_text()
    options = dict(name='retained_nanoflann', directory=str(_BUILD / 'cache'),
                   include_paths=(str(header.parent), str(source_dir)), std='c++17')
    _SETUP_TIMINGS['header_seconds'] = time.perf_counter() - start
    start = time.perf_counter()
    prebuild(code, decls=decls, **options)
    _SETUP_TIMINGS['prebuild_seconds'] = time.perf_counter() - start
    start = time.perf_counter()
    cppdef_cached(code, decls=decls, **options)
    _SETUP_TIMINGS['load_seconds'] = time.perf_counter() - start
    _NATIVE = cppyy.gbl.evaluation_nanoflann.RetainedIndex
    return _NATIVE


def _positions(values):
    array = np.require(values, dtype=np.float64, requirements=['C', 'A'])
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError('coordinates must have shape (n, 3)')
    if not np.all(np.isfinite(array)) or np.any(np.abs(array) > 1e150):
        raise ValueError('coordinates must be finite and have magnitude <= 1e150')
    return array


def _frames(values, count):
    array = np.asarray(values)
    if array.shape != (count,):
        raise ValueError('frames must have shape (n,)')
    # Empty Python sequences have floating dtype but contain no invalid IDs.
    if array.size and array.dtype.kind not in 'iu':
        raise ValueError('frame IDs must be integers')
    if np.any(array < 0) or np.any(array > 2**62):
        raise ValueError('frame IDs must be in [0, 2**62]')
    return np.require(array, dtype=np.int64, requirements=['C', 'A'])


def _integer(value, name, low, high):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f'{name} must be an integer')
    try:
        result = operator.index(value)
    except TypeError as exc:
        raise ValueError(f'{name} must be an integer') from exc
    if result < low or result > high:
        raise ValueError(f'{name} must be in [{low}, {high}]')
    return result


def _scalar(value, name):
    if np.ndim(value) != 0:
        raise ValueError(f'{name} must be a scalar')
    try:
        return float(value)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f'{name} must be numeric') from exc


def _pointer(array):
    # cppyy rejects empty NumPy buffers, even when the native count is zero.
    return array if array.size else np.zeros(1, dtype=array.dtype)


class Index:
    def __init__(self, xyz, frames, confidence):
        """Validate observations and copy them into a retained native tree."""
        self._index = None
        positions = _positions(xyz)
        frame_ids = _frames(frames, len(positions))
        weights = np.require(confidence, dtype=np.float64, requirements=['C', 'A'])
        if weights.shape != (len(positions),):
            raise ValueError('confidence must have shape (n,)')
        if not np.all(np.isfinite(weights)) or np.any((weights < 0) | (weights > 1)):
            raise ValueError('confidence must be finite and in [0, 1]')
        self._index = _native_class()(len(positions), _pointer(positions),
                                     _pointer(frame_ids), _pointer(weights))

    def query(self, xyz, frames, k=4, frame_gap=-1, min_confidence=0.0,
              max_distance=np.inf):
        """Return IDs, squared distances, counts and selected mean centroids."""
        if self._index is None:
            raise RuntimeError('index is closed')
        positions = _positions(xyz)
        frame_ids = _frames(frames, len(positions))
        k = _integer(k, 'k', 1, 4096)
        frame_gap = _integer(frame_gap, 'frame_gap', -1, 2**63 - 1)
        threshold = _scalar(min_confidence, 'min_confidence')
        radius = _scalar(max_distance, 'max_distance')
        if not np.isfinite(threshold) or not 0 <= threshold <= 1:
            raise ValueError('min_confidence must be finite and in [0, 1]')
        if np.isnan(radius) or radius < 0 or (np.isfinite(radius) and radius > 1e150):
            raise ValueError('max_distance must be in [0, 1e150] or positive infinity')
        count = len(positions)
        ids = np.empty((count, k), dtype=np.int64)
        distances = np.empty((count, k), dtype=np.float64)
        counts = np.empty(count, dtype=np.int64)
        centroids = np.empty((count, 3), dtype=np.float64)
        self._index.query(count, _pointer(positions), _pointer(frame_ids), k,
                          frame_gap, threshold, radius, _pointer(ids),
                          _pointer(distances), _pointer(counts), _pointer(centroids))
        return dict(ids=ids, distances_squared=distances, counts=counts,
                    centroids=centroids)

    def close(self):
        """Release native storage. Repeated calls succeed."""
        native = self._index
        self._index = None
        if native is not None:
            native.close()

    def __enter__(self):
        if self._index is None:
            raise RuntimeError('index is closed')
        return self

    def __exit__(self, *args):
        self.close()
