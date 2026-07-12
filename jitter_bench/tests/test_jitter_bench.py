#!/usr/bin/env python3
"""Smoke tests for the jitter_bench harness.

Fast, short-duration, and dependency-light so they run green in the default ``pixi run
test`` env: the pure-Python harness (recorder / stats / histogram / clock plumbing /
RT-knob attempts / background load) needs only numpy + ctypes. The cppyy C++-loop variant
(b) runs when cppyy + a working compile toolchain are present (they are in the default
env) and skips with a reason otherwise; the control_kit variant (c) skips unless
ros2_control is installed (the ``control`` env). Nothing here needs privilege.
"""
import os

import numpy as np
import pytest

from jitter_bench import harness
from jitter_bench.load import BackgroundLoad


# --- clock plumbing ---------------------------------------------------------
def test_clock_nanosleep_sleeps_to_deadline():
    """clock_nanosleep(TIMER_ABSTIME) returns 0 and wakes at/after the deadline."""
    deadline = harness.now_ns() + 3_000_000        # +3 ms
    rc = harness.clock_nanosleep_abs(deadline)
    woke = harness.now_ns()
    assert rc == 0
    assert woke >= deadline - 200_000              # not woken grossly early
    assert woke - deadline < 50_000_000            # and within a sane bound


def test_now_ns_is_monotonic():
    a = harness.now_ns()
    b = harness.now_ns()
    assert b >= a


# --- recorder ---------------------------------------------------------------
def test_recorder_captures_in_order():
    rec = harness.Recorder(8)
    for v in (10, 20, 30):
        rec.record(v)
    assert rec.count == 3
    assert not rec.wrapped
    assert list(rec.timestamps()) == [10, 20, 30]


def test_recorder_wraps_when_over_capacity():
    rec = harness.Recorder(2)
    for v in (1, 2, 3):
        rec.record(v)
    assert rec.wrapped
    assert rec.count == 3


# --- stats ------------------------------------------------------------------
def test_compute_stats_known_latencies():
    """Synthetic wake times with a KNOWN wakeup latency per cycle -> exact stats."""
    period_ns = 1_000_000                          # 1 ms (1 kHz)
    base = 5_000_000
    lat_ns = np.array([100_000, 200_000, 300_000, 400_000], dtype=np.int64)  # 100..400 us
    idx = np.arange(lat_ns.size)
    ts = base + (idx + 1) * period_ns + lat_ns
    stats = harness.compute_stats(ts, base, period_ns, drop_warmup=0)
    lat = stats["latency_us"]
    assert lat["min"] == pytest.approx(100.0)
    assert lat["max"] == pytest.approx(400.0)
    assert lat["mean"] == pytest.approx(250.0)
    assert stats["target_us"] == pytest.approx(1000.0)
    # latency ramps +100 us/cycle, so each interval is period+100 us -> +100 us period jitter
    assert stats["period_jitter_us"]["mean"] == pytest.approx(100.0)


def test_compute_stats_constant_latency_zero_period_jitter():
    """Constant wakeup latency -> perfectly periodic wakes -> ~zero period jitter."""
    period_ns = 1_000_000
    base = 0
    ts = base + (np.arange(6) + 1) * period_ns + 50_000     # constant +50 us offset
    stats = harness.compute_stats(ts, base, period_ns, drop_warmup=0)
    assert stats["latency_us"]["mean"] == pytest.approx(50.0)
    assert abs(stats["period_jitter_us"]["mean"]) < 1e-6


def test_compute_stats_drop_warmup():
    period_ns = 1_000_000
    base = 0
    lat_ns = np.array([9_000_000, 100_000, 100_000, 100_000], dtype=np.int64)  # cycle0 huge
    ts = base + (np.arange(lat_ns.size) + 1) * period_ns + lat_ns
    full = harness.compute_stats(ts, base, period_ns, drop_warmup=0)
    dropped = harness.compute_stats(ts, base, period_ns, drop_warmup=1)
    assert full["latency_us"]["max"] > 8000        # cycle-0 outlier present
    assert dropped["latency_us"]["max"] == pytest.approx(100.0)  # gone after drop
    assert dropped["dropped_warmup"] == 1


def test_histogram_renders():
    lat = np.array([0.5, 1.5, 3.0, 30.0, 300.0, 3000.0])
    out = harness.latency_histogram(lat)
    assert "bucket(us)" in out
    assert "total" in out


# --- RT knobs (attempt + record; never require privilege) -------------------
def test_mlockall_attempt_returns_tuple():
    ok, detail = harness.try_mlockall()
    assert isinstance(ok, bool) and isinstance(detail, str)
    if ok:
        harness.try_munlockall()


def test_scheduling_other_ok_and_fifo_reports():
    ok, detail = harness.apply_scheduling("other")
    assert ok
    ok_fifo, detail_fifo = harness.apply_scheduling("fifo", priority=80)
    # On a box without an rtprio grant this is denied; if granted it succeeds. Either way
    # it must return cleanly (never raise) so the Stage-1 rerun command is stable.
    assert isinstance(ok_fifo, bool) and isinstance(detail_fifo, str)


def test_timerslack_set_and_default():
    """PR_SET_TIMERSLACK is unprivileged; setting 1 ns takes, negative leaves the default."""
    prev = harness.get_timerslack_ns()
    try:
        ok, detail = harness.apply_timerslack(1)
        assert ok
        assert harness.get_timerslack_ns() == 1
        ok2, _ = harness.apply_timerslack(-1)      # negative == leave untouched
        assert not ok2
        assert harness.get_timerslack_ns() == 1    # unchanged by the -1 call
    finally:
        harness.apply_timerslack(prev if prev > 0 else 50000)   # restore


def test_affinity_roundtrip():
    original = os.sched_getaffinity(0)
    cpu = sorted(original)[0]
    ok, detail = harness.apply_affinity(cpu)
    assert ok
    os.sched_setaffinity(0, original)              # restore


# --- the fixed-rate loop (variants a1 / a2) ---------------------------------
@pytest.mark.parametrize("sleep_kind", ["clock_nanosleep", "time_sleep"])
def test_run_fixed_rate_short(sleep_kind):
    body, _ = harness.small_compute(10)
    rec, base, period_ns, n = harness.run_fixed_rate(
        1000.0, 0.08, sleep_kind, body=body)      # 80 ms -> ~80 cycles
    assert n == 80
    assert rec.count == 80
    stats = harness.compute_stats(rec.timestamps(), base, period_ns, drop_warmup=5)
    assert 500 < stats["achieved_hz"] < 1500       # roughly 1 kHz, tolerant
    assert stats["latency_us"]["min"] >= -50       # not woken absurdly early


# --- background load --------------------------------------------------------
def test_background_load_spawn_and_kill():
    load = BackgroundLoad(n=2)
    load.start()
    try:
        assert len(load._procs) == 2
        assert all(p.poll() is None for p in load._procs)   # alive
        assert "busy-loop" in load.describe()
    finally:
        load.stop()
    assert all(p.poll() is not None for p in load._procs)    # dead


# --- variant b: the cppyy_kit C++ loop (runs where cppyy compiles) ----------
def test_cpp_loop_variant():
    try:
        import cppyy  # noqa: F401
        from jitter_bench import cpp_loop
    except Exception as exc:                       # pragma: no cover
        pytest.skip("cppyy unavailable: %s" % exc)
    try:
        rec, base, period_ns, n = cpp_loop.run_cpp_loop(
            1000.0, 0.05, compute_iters=10, use_nogil=True)   # 50 ms
    except Exception as exc:                       # compile-toolchain / cppyy hiccup
        pytest.skip("cppyy C++ loop unavailable: %s" % str(exc).splitlines()[0])
    assert n == 50
    assert rec.count == 50
    assert base > 0
    stats = harness.compute_stats(rec.timestamps(), base, period_ns, drop_warmup=5)
    assert 500 < stats["achieved_hz"] < 1500
    # C++ writes real timestamps in ascending order
    ts = rec.timestamps()
    assert np.all(np.diff(ts) >= 0)


# --- variant c: control_kit loop (control env only) -------------------------
def test_control_loop_variant():
    from jitter_bench.control_loop import have_control, ControlLoop, ControlUnavailable
    if not have_control():
        pytest.skip("ros2_control not installed (use the control env)")
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    try:
        loop = ControlLoop(1000.0, controller="python").setup()
    except ControlUnavailable as exc:              # pragma: no cover
        pytest.skip(str(exc))
    try:
        rec, base, period_ns, n = harness.run_fixed_rate(
            1000.0, 0.1, "clock_nanosleep", body=loop.body)   # 100 ms
        stats = harness.compute_stats(rec.timestamps(), base, period_ns, drop_warmup=5)
        assert stats["cycles_used"] > 50
        assert 500 < stats["achieved_hz"] < 1500
    finally:
        loop.teardown()
