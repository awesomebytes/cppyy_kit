#!/usr/bin/env python3
"""Benchmark: cppyy element access vs. as_array() over LaserScan.ranges.

Needs a real NativeSession bringup (sensor_msgs' generated headers), so it
runs isolated in its own subprocess like the other NativeSession-based
helpers in this suite (see test_array_view.py).

Prints one line of numbers per LaserScan size and asserts the *direction* of
the win (as_array indexed access faster than raw cppyy indexing, bulk sum
much faster than either), not tight absolute thresholds -- the field-access-
investigation.md numbers (134ns cppyy vs 32ns numpy indexed) were measured on
a specific box and cppyy JIT warmup noise makes tight bounds flaky in CI.
"""
import time

from rclcpp_kit.array_view import as_array
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


def _time_ns(fn, repeats):
    # Warm up (first call pays cppyy's one-time call-wrapper JIT cost).
    fn()
    start = time.perf_counter_ns()
    for _ in range(repeats):
        fn()
    end = time.perf_counter_ns()
    return (end - start) / repeats


def _bench_one_size(cpp_laser_scan, size, repeats=20000):
    scan = cpp_laser_scan()
    scan.ranges.resize(size)
    for i in range(size):
        scan.ranges[i] = float(i) * 0.25
    mid = size // 2

    def cppyy_index():
        return float(scan.ranges[mid])

    def numpy_index():
        arr = as_array(scan.ranges)
        return float(arr[mid])

    arr = as_array(scan.ranges)

    def numpy_index_cached():
        return float(arr[mid])

    def numpy_sum():
        return float(arr.sum())

    cppyy_ns = _time_ns(cppyy_index, repeats)
    numpy_ns = _time_ns(numpy_index, repeats)
    numpy_cached_ns = _time_ns(numpy_index_cached, repeats)
    sum_ns = _time_ns(numpy_sum, repeats)

    print(
        "ARRAY_VIEW_BENCHMARK size=%d cppyy_index_ns=%.1f "
        "as_array_index_ns=%.1f as_array_cached_index_ns=%.1f "
        "as_array_sum_ns=%.1f" % (size, cppyy_ns, numpy_ns, numpy_cached_ns, sum_ns),
        flush=True,
    )

    # The cached-view indexed access (as_array() called once, then indexed
    # repeatedly -- the realistic usage pattern) must beat raw cppyy
    # indexing; a generous 1.2x margin absorbs measurement noise while still
    # catching a regression that erases the win entirely.
    assert numpy_cached_ns < cppyy_ns / 1.2, (
        "as_array indexed access (%.1fns) did not beat cppyy indexing "
        "(%.1fns) for size=%d" % (numpy_cached_ns, cppyy_ns, size))
    # A full-array reduction must be far cheaper per element than any
    # element-by-element loop over the same size.
    assert sum_ns < cppyy_ns * size / 4, (
        "as_array().sum() (%.1fns) was not far below an equivalent "
        "element-by-element cppyy loop estimate for size=%d" % (sum_ns, size))


def main():
    with native(["array-view-benchmark"]) as session:
        session.create_node("array_view_benchmark_test")
        CppLaserScan = load_message_type("sensor_msgs", "LaserScan").cpp_type
        for size in (360, 1080):
            _bench_one_size(CppLaserScan, size)
        print("ARRAY_VIEW_BENCHMARK_OK", flush=True)


if __name__ == "__main__":
    main()
