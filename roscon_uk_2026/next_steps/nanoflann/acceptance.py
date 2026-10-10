"""Independent NumPy oracle; NN_SOLUTION can point to an agent's solution."""
import gc
import importlib.util
import os
from pathlib import Path

import numpy as np

solution = Path(os.environ.get("NN_SOLUTION", Path(__file__).with_name("index.py")))
spec = importlib.util.spec_from_file_location("nn_solution", solution)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Index = module.Index


def reference(points, point_frames, confidence, queries, frames, k,
              frame_gap=-1, min_confidence=0., max_distance=np.inf):
    ids = np.full((len(queries), k), -1, dtype=np.int64)
    d2 = np.full((len(queries), k), np.inf)
    centroids = np.full((len(queries), 3), np.nan)
    counts = np.zeros(len(queries), dtype=np.int64)
    for j, query in enumerate(queries):
        squared = np.sum((points-query)**2, axis=1)
        eligible = (squared <= max_distance**2) & (confidence >= min_confidence)
        if frame_gap >= 0:
            eligible &= np.abs(point_frames-frames[j]) > frame_gap
        candidates = np.flatnonzero(eligible)
        chosen = candidates[np.lexsort((candidates, squared[candidates]))[:k]]
        count = len(chosen)
        counts[j] = count
        ids[j, :count] = chosen
        d2[j, :count] = squared[chosen]
        if count:
            centroids[j] = points[chosen].mean(axis=0)
    return dict(ids=ids, distances_squared=d2, counts=counts, centroids=centroids)


def compare(got, expected):
    np.testing.assert_array_equal(got["ids"], expected["ids"])
    np.testing.assert_array_equal(got["counts"], expected["counts"])
    np.testing.assert_allclose(got["distances_squared"], expected["distances_squared"],
                               rtol=2e-14, atol=1e-14)
    np.testing.assert_allclose(got["centroids"], expected["centroids"],
                               rtol=2e-14, atol=1e-14, equal_nan=True)


def rejected(operation):
    try:
        operation()
    except Exception:
        return
    raise AssertionError("Invalid input was accepted")


def run():
    rng = np.random.default_rng(4026)
    cases = 0
    # Integer coordinates force exact ties across many tree leaves, including
    # duplicate points at zero radius. Original indices must resolve those ties.
    for points in (rng.normal(size=(257, 3)), rng.integers(-2, 3, (513, 3)).astype(float),
                   np.zeros((65, 3)), np.empty((0, 3))):
        pf = np.arange(len(points), dtype=np.int64)
        confidence = rng.integers(0, 3, len(points))/2
        queries = np.vstack((np.zeros((1, 3)), rng.integers(-2, 3, (19, 3))))
        frames = np.arange(len(queries), dtype=np.int64)
        with Index(points, pf, confidence) as index:
            for k in (1, 4, 17, 600):
                for gap, minimum, radius in ((-1, 0., np.inf), (3, .5, 2.),
                                             (1000, 1., 0.), (-1, 0., 0.)):
                    args = dict(k=k, frame_gap=gap, min_confidence=minimum, max_distance=radius)
                    compare(index.query(queries, frames, **args),
                            reference(points, pf, confidence, queries, frames, **args))
                    cases += 1
            empty = index.query(np.empty((0, 3)), np.empty(0, dtype=np.int64), k=3)
            assert empty["ids"].shape == (0, 3)
    # Required eligible neighbor is beyond the nearest unfiltered k candidates.
    points = np.column_stack((np.arange(100.), np.zeros((100, 2))))
    pf = np.arange(100, dtype=np.int64)
    confidence = np.ones(100)
    with Index(points, pf, confidence) as index:
        got = index.query([[0., 0., 0.]], [0], k=2, frame_gap=50)
        assert got["ids"].tolist() == [[51, 52]]
        assert got["distances_squared"].tolist() == [[2601., 2704.]]
        np.testing.assert_equal(np.sqrt(got["distances_squared"]), [[51., 52.]])
    # Native owns a copy; Python can modify, resize and release its input arrays.
    points = np.array([[1., 0., 0.], [2., 0., 0.]])
    pf = np.array([0, 1], dtype=np.int64)
    confidence = np.ones(2)
    index = Index(points, pf, confidence)
    points[:] = 999
    points.resize((100, 3), refcheck=False)
    pf[:] = 9
    confidence[:] = 0
    del points, pf, confidence
    gc.collect()
    for _ in range(20):
        assert index.query([[0., 0., 0.]], [0], k=1)["ids"].tolist() == [[0]]
        assert index.query([[0., 0., 0.]], [0], k=1)["distances_squared"].tolist() == [[1.]]
    retained_output = index.query([[0., 0., 0.]], [0], k=1)
    index.query([[10., 0., 0.]], [0], k=1)
    assert retained_output["ids"].tolist() == [[0]]
    for bad in (np.nan, np.inf, -np.inf, 1e151):
        rejected(lambda: Index([[bad, 0., 0.]], [0], [1.]))
        rejected(lambda: index.query([[bad, 0., 0.]], [0]))
    for bad in (np.nan, np.inf, -0.1, 1.1):
        rejected(lambda: Index([[0., 0., 0.]], [0], [bad]))
        rejected(lambda: index.query([[0., 0., 0.]], [0], min_confidence=bad))
    for bad in (-1., np.nan, -np.inf, 1e151):
        rejected(lambda: index.query([[0., 0., 0.]], [0], max_distance=bad))
    for bad in (0, -1, 4097, True, 1.5):
        rejected(lambda: index.query([[0., 0., 0.]], [0], k=bad))
    for bad_frames in ([1.5], [-1], [2**63], []):
        rejected(lambda: Index([[0., 0., 0.]], bad_frames, [1.]))
        rejected(lambda: index.query([[0., 0., 0.]], bad_frames))
    rejected(lambda: index.query([[0., 0.]], [0]))
    rejected(lambda: index.query([[0., 0., 0.]], [0], frame_gap=-2))
    # A strided and read-only input is explicitly normalized into a safe buffer.
    xyz = np.arange(18., dtype=float).reshape(3, 6)[:, ::2]
    xyz.flags.writeable = False
    index.query(xyz, np.arange(3), k=2)
    # A C-contiguous array can still have an unaligned data pointer. Normalize
    # all typed inputs before C++ accesses double*/int64_t* storage.
    unaligned_xyz = np.ndarray((2, 3), dtype=np.float64, buffer=bytearray(49), offset=1)
    unaligned_frames = np.ndarray(2, dtype=np.int64, buffer=bytearray(17), offset=1)
    unaligned_confidence = np.ndarray(2, dtype=np.float64, buffer=bytearray(17), offset=1)
    unaligned_xyz[:] = [[0., 0., 0.], [1., 0., 0.]]
    unaligned_frames[:] = [0, 1]
    unaligned_confidence[:] = [1., .5]
    assert not unaligned_xyz.flags.aligned
    with Index(unaligned_xyz, unaligned_frames, unaligned_confidence) as unaligned_index:
        compare(unaligned_index.query(unaligned_xyz, unaligned_frames, k=2),
                reference(unaligned_xyz, unaligned_frames, unaligned_confidence,
                          unaligned_xyz, unaligned_frames, k=2))
    index.close()
    index.close()
    assert retained_output["distances_squared"].tolist() == [[1.]]
    rejected(lambda: index.query([[0., 0., 0.]], [0]))
    for _ in range(30):
        with Index([[0., 0., 0.]], [0], [1.]) as temporary:
            assert temporary.query([[0., 0., 0.]], [0])["counts"].tolist() == [1]
    print(f"PASS: {cases} oracle comparisons; eligibility, distance units, owned lifetime, "
          "empty batches, invalid inputs, repeated close and teardown")


if __name__ == "__main__":
    run()
