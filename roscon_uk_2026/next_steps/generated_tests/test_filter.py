"""Generated input properties and generated sequences against compiled C++."""

from decimal import Decimal, localcontext

import numpy as np
import pytest
from hypothesis import event, example, given, settings, strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, precondition, rule

from roscon_uk_2026.next_steps.reverse_core import FilterConfig, PoseFilter, native_namespace
from oracle import ReferenceFilter, assert_snapshot

RUN = settings(max_examples=150, deadline=None, derandomize=True, database=None)
COORDINATE = st.floats(-1e6, 1e6, allow_nan=False, allow_infinity=False)
POINT = st.tuples(COORDINATE, COORDINATE, COORDINATE)
CONFIG = st.sampled_from([
    FilterConfig(tau_s=0.08, max_gap_s=0.5),
    FilterConfig(tau_s=0.001, max_gap_s=0.001),
    FilterConfig(tau_s=0.25, max_gap_s=2.0, frame_id="map/base"),
])


@st.composite
def batch(draw):
    points = draw(st.lists(POINT, max_size=40))
    increments = draw(st.lists(st.integers(1, 3_000_000_000),
                               min_size=len(points), max_size=len(points)))
    timestamp = draw(st.integers(-10**15, 10**15))
    times = []
    for step in increments:
        timestamp += step
        times.append(timestamp)
    return np.array(times, dtype=np.int64), np.array(points, dtype=np.float64).reshape(-1, 3)


@RUN
@given(config=CONFIG, inputs=batch())
def test_independent_reference(config, inputs):
    times, points = inputs
    reference = ReferenceFilter(config.model_dump())
    with PoseFilter(config) as native:
        actual = native.process(times, points)
        expected = reference.process(times, points)
        np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-9)
        assert_snapshot(native.snapshot(), reference.snapshot())
        assert actual.dtype == np.float64 and actual.shape == points.shape
        assert actual.flags.owndata and actual.flags.c_contiguous
        assert not np.shares_memory(actual, points)
    event("empty" if len(times) == 0 else "nonempty")


@RUN
@given(config=CONFIG, inputs=batch(), cuts=st.lists(st.integers(0, 40), max_size=12))
def test_chunking(config, inputs, cuts):
    times, points = inputs
    boundaries = [0] + sorted(min(cut, len(times)) for cut in cuts) + [len(times)]
    with PoseFilter(config) as whole, PoseFilter(config) as chunks:
        expected = whole.process(times, points)
        results = [chunks.process(times[a:b], points[a:b])
                   for a, b in zip(boundaries, boundaries[1:])]
        actual = np.concatenate(results)
        np.testing.assert_array_equal(actual, expected)
        assert chunks.snapshot() == whole.snapshot()


@RUN
@given(config=CONFIG, first=batch(), second=batch())
def test_reset_isolation(config, first, second):
    with PoseFilter(config) as reused, PoseFilter(config) as fresh:
        reused.process(*first)
        reused.reset()
        reused.reset()
        np.testing.assert_array_equal(reused.process(*second), fresh.process(*second))
        assert reused.snapshot() == fresh.snapshot()


@RUN
@given(points=st.lists(POINT, min_size=3, max_size=3))
def test_gap_threshold(points):
    config = FilterConfig(tau_s=0.25, max_gap_s=0.5)
    times = np.array([0, 500_000_000, 1_000_000_001], dtype=np.int64)
    positions = np.array(points, dtype=np.float64)
    model = ReferenceFilter(config.model_dump())
    with PoseFilter(config) as native:
        np.testing.assert_allclose(native.process(times, positions), model.process(times, positions),
                                   rtol=1e-12, atol=1e-9)
        assert native.snapshot()["gap_resets"] == 1


@RUN
@example(first=-np.finfo(np.float64).max, second=np.finfo(np.float64).max,
         step=100_000_000)
@example(first=np.finfo(np.float64).max, second=np.finfo(np.float64).max,
         step=500_000_000)
@given(first=st.floats(allow_nan=False, allow_infinity=False),
       second=st.floats(allow_nan=False, allow_infinity=False),
       step=st.sampled_from([1, 100_000_000, 500_000_000]))
def test_full_finite_float_range(first, second, step):
    # Decimal supplies an independent high-precision convex combination.
    # A scale-based absolute tolerance accounts for cancellation near zero.
    with localcontext() as context:
        context.prec = 80
        weight = 1 - (-Decimal(step) / Decimal(10**9) / Decimal.from_float(0.08)).exp()
        expected = float((1 - weight) * Decimal.from_float(first)
                         + weight * Decimal.from_float(second))
    times = np.array([0, step], dtype=np.int64)
    points = np.array([[first] * 3, [second] * 3], dtype=np.float64)
    with PoseFilter() as native:
        result = native.process(times, points)
        assert np.isfinite(result).all()
        np.testing.assert_allclose(result[-1], expected, rtol=1e-12,
                                   atol=max(abs(first), abs(second)) * 1e-15 + 1e-300)


@RUN
@given(kind=st.sampled_from(["repeat", "decrease", "across", "nan", "inf"]),
       points=st.lists(POINT, min_size=3, max_size=3))
def test_native_rejection_is_atomic(kind, points):
    # Bypass Python validation with valid buffer layouts, so this check reaches
    # the separately compiled C++ validation rather than only the adapter.
    namespace = native_namespace()
    native = namespace.PoseEstimator(namespace.Config())
    seed_time = np.array([0], dtype=np.int64)
    seed_point = np.zeros(3, dtype=np.float64)
    native.process(seed_time, seed_point, 1, np.empty(3, dtype=np.float64))
    times = np.array([1, 2, 3], dtype=np.int64)
    positions = np.array(points, dtype=np.float64).reshape(-1)
    if kind == "repeat":
        times[-1] = 2
    elif kind == "decrease":
        times[-1] = 1
    elif kind == "across":
        times[0] = 0
    else:
        positions[-1] = float(kind)
    before = native.snapshot()
    output = np.full(9, -123.0, dtype=np.float64)
    try:
        with pytest.raises(Exception) as caught:
            native.process(times, positions, 3, output)
        assert type(caught.value).__cpp_name__ == "std::invalid_argument"
        after = native.snapshot()
        assert after.initialized == before.initialized
        assert after.gap_resets == before.gap_resets
        assert after.samples_processed == before.samples_processed == 1
        assert after.last_timestamp_ns == before.last_timestamp_ns == 0
        assert list(after.position_m) == list(before.position_m)
        np.testing.assert_array_equal(output, -123.0)
        native.process(np.array([4], dtype=np.int64), np.ones(3, dtype=np.float64), 1, output[:3])
        assert native.snapshot().samples_processed == 2
    finally:
        del native


def test_int64_boundary_and_buffer_policy():
    times = np.array([np.iinfo(np.int64).min, -1, 0, np.iinfo(np.int64).max], dtype=np.int64)
    points = np.arange(12, dtype=np.float64).reshape(4, 3)
    with PoseFilter() as native:
        result = native.process(times, points)
        assert np.isfinite(result).all()
        assert native.snapshot()["gap_resets"] == 2
    # Create strided buffers and then read-only contiguous copies.
    backing = np.zeros((8, 6), dtype=np.float64)
    backing[::2, ::2] = points
    time_backing = np.zeros(8, dtype=np.int64)
    time_backing[::2] = times
    unaligned_times = np.ndarray(times.shape, dtype=np.int64, buffer=bytearray(times.nbytes + 1), offset=1)
    unaligned_points = np.ndarray(points.shape, dtype=np.float64, buffer=bytearray(points.nbytes + 1), offset=1)
    unaligned_times[:] = times
    unaligned_points[:] = points
    assert not unaligned_times.flags.aligned and not unaligned_points.flags.aligned
    for t, p in ((time_backing[::2], backing[::2, ::2]), (times, points),
                 (unaligned_times, unaligned_points)):
        t.flags.writeable = False
        p.flags.writeable = False
        with PoseFilter() as native:
            np.testing.assert_array_equal(native.process(t, p), result)
        np.testing.assert_array_equal(t, times)
        np.testing.assert_array_equal(p, points)


class FilterSequence(RuleBasedStateMachine):
    def __init__(self):
        super().__init__()
        self.config = FilterConfig(tau_s=0.08, max_gap_s=0.5)
        self.native = PoseFilter(self.config)
        self.model = ReferenceFilter(self.config.model_dump())

    @rule(points=st.lists(POINT, min_size=1, max_size=6),
          step=st.sampled_from([1, 10_000_000, 500_000_000, 500_000_001, 2_000_000_000]))
    def feed(self, points, step):
        first = -1_000_000_000 if self.model.last is None else self.model.last
        times = np.array([first + (i + 1) * step for i in range(len(points))], dtype=np.int64)
        positions = np.asarray(points, dtype=np.float64)
        np.testing.assert_allclose(self.native.process(times, positions),
                                   self.model.process(times, positions), rtol=1e-12, atol=1e-9)
        event("feed:gap" if step > 500_000_000 else "feed:smooth")

    @rule()
    def reset(self):
        self.native.reset()
        self.model.reset()
        event("reset")

    @rule()
    def empty(self):
        before = self.native.snapshot()
        result = self.native.process(np.empty(0, dtype=np.int64), np.empty((0, 3), dtype=np.float64))
        assert result.shape == (0, 3)
        assert self.native.snapshot() == before
        event("empty")

    @precondition(lambda self: self.model.last is not None)
    @rule(delta=st.sampled_from([0, -1, -1_000_000_000]))
    def invalid_across_calls(self, delta):
        before = self.native.snapshot()
        times = np.array([self.model.last + delta], dtype=np.int64)
        with pytest.raises(ValueError):
            self.native.process(times, np.zeros((1, 3), dtype=np.float64))
        assert self.native.snapshot() == before
        event("reject:across-repeat" if delta == 0 else "reject:across-decrease")

    @rule(kind=st.sampled_from(["repeat", "decrease", "nan", "inf", "-inf", "shape", "length", "dtype"]))
    def invalid_batch(self, kind):
        start = -1_000_000_000 if self.model.last is None else self.model.last
        times = np.array([start + 1, start + 2, start + 3], dtype=np.int64)
        points = np.zeros((3, 3), dtype=np.float64)
        exception = ValueError
        if kind == "repeat":
            times[-1] = times[-2]
        elif kind == "decrease":
            times[-1] = times[-2] - 1
        elif kind in ("nan", "inf", "-inf"):
            points[-1, -1] = float(kind)
        elif kind == "shape":
            points = np.zeros((3, 2), dtype=np.float64)
        elif kind == "length":
            points = points[:-1]
        else:
            points = points.astype(np.float32)
            exception = TypeError
        before = self.native.snapshot()
        with pytest.raises(exception):
            self.native.process(times, points)
        assert self.native.snapshot() == before
        event("reject:" + kind)

    @invariant()
    def state_matches_reference(self):
        assert_snapshot(self.native.snapshot(), self.model.snapshot())

    def teardown(self):
        self.native.close()


TestFilterSequence = FilterSequence.TestCase
TestFilterSequence.settings = settings(max_examples=100, stateful_step_count=35,
                                      deadline=None, derandomize=True, database=None)


@pytest.mark.parametrize("method", ["process", "reset", "snapshot", "__enter__"])
def test_closed_operations(method):
    native = PoseFilter()
    native.close()
    native.close()
    with pytest.raises(RuntimeError):
        args = (np.empty(0, dtype=np.int64), np.empty((0, 3), dtype=np.float64)) if method == "process" else ()
        getattr(native, method)(*args)


@pytest.mark.parametrize("kind", ["timestamps-list", "points-list", "timestamp-dtype", "timestamp-shape", "points-rank", "non-native-endian"])
def test_binding_rejects_invalid_buffers(kind):
    times = np.array([1, 2], dtype=np.int64)
    points = np.zeros((2, 3), dtype=np.float64)
    error = TypeError
    if kind == "timestamps-list":
        times = times.tolist()
    elif kind == "points-list":
        points = points.tolist()
    elif kind == "timestamp-dtype":
        times = times.astype(np.int32)
    elif kind == "timestamp-shape":
        times = times.reshape(2, 1)
        error = ValueError
    elif kind == "points-rank":
        points = points.reshape(6)
        error = ValueError
    else:
        points = points.astype(">f8")
    with PoseFilter() as native:
        before = native.snapshot()
        with pytest.raises(error):
            native.process(times, points)
        assert native.snapshot() == before
