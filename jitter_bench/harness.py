#!/usr/bin/env python3
"""Timing loop, statistics, and system settings for jitter_bench.

This module provides a fixed-rate loop, wakeup statistics, and best-effort calls for
memory locking, CPU affinity, scheduling policy, and timer slack. Per-cycle timestamps
use NumPy buffers and ctypes. The other jitter_bench modules use this harness.

The harness uses `CLOCK_MONOTONIC` for both deadlines and timestamps. On Linux,
CPython's `perf_counter` uses this clock. `clock_nanosleep` also uses it, so wakeup
latency is the measured wake time minus its deadline. `cyclictest` uses the same clock
by default. The report also gives period jitter: each interval between wakes minus the
requested period. See control_kit REPORT section 4 for that metric.

The calls used here do not need privilege except for `SCHED_FIFO`, which requires an
rtprio grant. The harness tries the requested policy and records any denial.
"""
import ctypes
import os
import time

import numpy as np

# --- libc / clock plumbing -------------------------------------------------
_LIBC = ctypes.CDLL("libc.so.6", use_errno=True)

CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1
MCL_CURRENT = 1
MCL_FUTURE = 2
PR_SET_TIMERSLACK = 29
PR_GET_TIMERSLACK = 30


class _timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


def now_ns():
    """Return the current time in nanoseconds from `CLOCK_MONOTONIC`."""
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def clock_nanosleep_abs(deadline_ns):
    """Sleep until an absolute `CLOCK_MONOTONIC` deadline.

    Returns the libc return code: 0 on success, non-zero on error or interruption."""
    ts = _timespec(deadline_ns // 1_000_000_000, deadline_ns % 1_000_000_000)
    return _LIBC.clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ctypes.byref(ts), None)


def sleep_until_time_sleep(deadline_ns):
    """Sleep for the time remaining before `deadline_ns` using `time.sleep`.

    Recomputing the remaining time avoids schedule drift and allows comparison with
    `clock_nanosleep`."""
    remaining = deadline_ns - now_ns()
    if remaining > 0:
        time.sleep(remaining / 1e9)


# The two Python sleep mechanisms, by key (variant b runs its wait in C++).
SLEEPERS = {
    "clock_nanosleep": clock_nanosleep_abs,
    "time_sleep": sleep_until_time_sleep,
}


# --- real-time knobs (all best-effort, all record their outcome) ------------
def try_mlockall():
    """Lock current and future pages in RAM. Returns `(ok, detail)`.

    This machine allows unprivileged locking under its approximately 8 GB memlock limit."""
    rc = _LIBC.mlockall(MCL_CURRENT | MCL_FUTURE)
    if rc == 0:
        return True, "mlockall(MCL_CURRENT|MCL_FUTURE) ok"
    return False, "mlockall failed (errno %d: %s)" % (
        ctypes.get_errno(), os.strerror(ctypes.get_errno()))


def try_munlockall():
    _LIBC.munlockall()


def apply_affinity(cpu):
    """Pin this process to one CPU. Returns `(ok, detail)`."""
    if cpu is None:
        return False, "affinity not set (running on all %d cpus)" % len(os.sched_getaffinity(0))
    try:
        os.sched_setaffinity(0, {int(cpu)})
        return True, "pinned to cpu %d" % int(cpu)
    except (OSError, ValueError) as exc:
        return False, "sched_setaffinity(cpu=%r) failed: %s" % (cpu, exc)


def apply_scheduling(policy, priority=80):
    """Set the scheduling policy and return `(ok, detail)`.

    `other` uses the default CFS policy. `fifo` and `rr` require an rtprio grant.
    Permission errors are recorded. The `nice` value is adjusted when running under
    SCHED_OTHER."""
    policy = (policy or "other").lower()
    if policy == "other":
        detail = "SCHED_OTHER (default CFS)"
        try:
            os.nice(-5)
            detail += "; nice -5 applied"
        except (OSError, PermissionError):
            detail += "; nice unchanged (needs privilege)"
        return True, detail
    const = {"fifo": os.SCHED_FIFO, "rr": os.SCHED_RR}.get(policy)
    if const is None:
        return False, "unknown scheduling policy %r" % policy
    try:
        os.sched_setscheduler(0, const, os.sched_param(int(priority)))
        return True, "SCHED_%s prio %d" % (policy.upper(), priority)
    except (PermissionError, OSError) as exc:
        return False, ("SCHED_%s DENIED (%s) -- needs an rtprio grant (ulimit -r is %s). "
                       "Falling back to SCHED_OTHER for this run."
                       % (policy.upper(), exc, _rtprio_limit()))


def _rtprio_limit():
    try:
        import resource
        soft, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
        return str(soft)
    except Exception:
        return "?"


def get_timerslack_ns():
    """Return this thread's timer slack in nanoseconds. Linux defaults to 50,000 ns."""
    return _LIBC.prctl(PR_GET_TIMERSLACK, 0, 0, 0, 0)


def apply_timerslack(ns):
    """Set the calling thread's timer slack (ns). Returns ``(ok, detail)``. **Unprivileged.**

    Timer slack is how long the kernel may *defer* a timer (nanosleep / clock_nanosleep /
    futex / poll) to batch wakeups; the Linux default is **50 µs**, which rounds up the
    short absolute-deadline sleeps of a 1 kHz loop and is a dominant contributor to the
    *median* wakeup latency. Tightening it to 1 ns is the first progressive-tuning step and
    needs no privilege. ``ns < 0`` leaves the OS default untouched (to measure the untuned
    contrast). All variants run their loop on the calling thread, so setting it once here
    covers a1/a2/b/c."""
    prev = get_timerslack_ns()
    if ns is None or ns < 0:
        return False, "left at OS default (%d ns)" % prev
    rc = _LIBC.prctl(PR_SET_TIMERSLACK, ctypes.c_ulong(int(ns)), 0, 0, 0)
    if rc != 0:
        return False, "PR_SET_TIMERSLACK failed (errno %d: %s)" % (
            ctypes.get_errno(), os.strerror(ctypes.get_errno()))
    return True, "set to %d ns (was %d ns; Linux default 50000)" % (get_timerslack_ns(), prev)


# --- recorder: one preallocated buffer, no per-cycle allocation -------------
class Recorder:
    """Store wake timestamps in a preallocated `int64` buffer.

    Python calls `record()` to store each value. The C++ loop writes to the buffer by
    address. If the buffer fills, it wraps and sets `wrapped`. Fixed-duration runs are
    sized to fit in the buffer."""

    def __init__(self, capacity):
        self.buf = np.zeros(int(capacity), dtype=np.int64)
        self.capacity = int(capacity)
        self.count = 0
        self.wrapped = False

    def record(self, ts_ns):
        i = self.count
        if i >= self.capacity:
            self.wrapped = True
            i = i % self.capacity
        self.buf[i] = ts_ns
        self.count += 1

    def timestamps(self):
        """Return recorded wake timestamps in order for a non-wrapping run."""
        if self.wrapped:
            return self.buf.copy()
        return self.buf[:self.count].copy()


# --- stats -----------------------------------------------------------------
_PCTS = (0, 50, 90, 99, 99.9, 100)


def compute_stats(timestamps_ns, base_ns, period_ns, drop_warmup=0):
    """Compute wakeup latency and period jitter from recorded timestamps.

    `base_ns` is the loop start time. Deadline `i` is `base + (i + 1) * period`.
    Returns the latency and jitter statistics, late-cycle counts, and arrays used for
    histograms. `drop_warmup` excludes the first N timestamps from the statistics."""
    ts = np.asarray(timestamps_ns, dtype=np.int64)
    n_total = ts.size
    if drop_warmup and n_total > drop_warmup:
        ts = ts[drop_warmup:]
        idx0 = drop_warmup
    else:
        idx0 = 0
    n = ts.size
    if n < 2:
        raise ValueError("need >=2 timestamps for stats (got %d)" % n)
    # deadline[k] for the k-th *kept* sample (k-th sample is cycle idx0+k, whose
    # programmed absolute wake is base + (idx0+k+1)*period).
    cycle_index = np.arange(idx0, idx0 + n, dtype=np.int64)
    deadlines = base_ns + (cycle_index + 1) * period_ns
    latency_us = (ts - deadlines) / 1000.0                     # wakeup latency vs grid
    period_us = np.diff(ts) / 1000.0                            # consecutive intervals
    target_us = period_ns / 1000.0
    period_err_us = period_us - target_us                      # signed period jitter
    late_1_5x = int(np.count_nonzero(period_us > 1.5 * target_us))
    overrun = int(np.count_nonzero(latency_us > target_us))    # woke a full period late

    def _p(arr):
        vals = np.percentile(arr, _PCTS)
        return {("p%s" % p).replace(".0", ""): float(v) for p, v in zip(_PCTS, vals)}

    return {
        "cycles": int(n_total),
        "cycles_used": int(n),
        "dropped_warmup": int(idx0),
        "target_us": float(target_us),
        "achieved_hz": float((n - 1) / ((ts[-1] - ts[0]) / 1e9)) if ts[-1] > ts[0] else 0.0,
        "late_1_5x": late_1_5x,
        "late_1_5x_pct": 100.0 * late_1_5x / max(1, n - 1),
        "overrun_full_period": overrun,
        "latency_us": {
            "min": float(latency_us.min()), "mean": float(latency_us.mean()),
            "std": float(latency_us.std()), "max": float(latency_us.max()),
            **_p(latency_us),
        },
        "period_jitter_us": {
            "min": float(period_err_us.min()), "mean": float(period_err_us.mean()),
            "std": float(period_err_us.std()), "max": float(period_err_us.max()),
            "abs_p99": float(np.percentile(np.abs(period_err_us), 99)),
            "abs_p999": float(np.percentile(np.abs(period_err_us), 99.9)),
        },
        "_latency_us_arr": latency_us,       # kept for the histogram; not JSON-serialized
    }


# Latency histogram bucket edges (µs). Log-ish; open-ended top bucket.
_HIST_EDGES = [0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]


def latency_histogram(latency_us, edges=_HIST_EDGES, width=40):
    """Format an ASCII histogram of wakeup latency in µs.

    Shows counts, bars, and cumulative percentages. Negative values appear in ``<0``."""
    lat = np.asarray(latency_us, dtype=np.float64)
    n = lat.size
    below = int(np.count_nonzero(lat < edges[0]))
    counts = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        counts.append(int(np.count_nonzero((lat >= lo) & (lat < hi))))
    above = int(np.count_nonzero(lat >= edges[-1]))
    rows = [("<%g" % edges[0], below)]
    for lo, hi in zip(edges[:-1], edges[1:]):
        rows.append(("%g-%g" % (lo, hi), counts.pop(0)))
    rows.append((">=%g" % edges[-1], above))
    peak = max((c for _, c in rows), default=1) or 1
    lines = ["  %-12s %9s %6s  %s" % ("bucket(us)", "count", "%", "histogram")]
    cum = 0
    for label, c in rows:
        cum += c
        bar = "#" * int(round(width * c / peak))
        lines.append("  %-12s %9d %5.1f%%  %s" % (label, c, 100.0 * c / max(1, n), bar))
    lines.append("  %-12s %9d %5.1f%%  (cumulative)" % ("total", n, 100.0 * cum / max(1, n)))
    return "\n".join(lines)


# --- the fixed-rate loop (Python variants a1/a2 and the driver for c) -------
def run_fixed_rate(rate_hz, duration_s, sleep_kind="clock_nanosleep",
                   body=None, recorder=None):
    """Run at ``rate_hz`` for ``duration_s`` using absolute deadlines and record wake times.

    ``sleep_kind`` selects the wait function in ``SLEEPERS``. ``body(i)`` is the work
    performed each cycle; omit it for a timer-only loop. Record wake time before running
    the body so latency measures scheduling rather than computation. Returns
    ``(recorder, base_ns, period_ns, n)``."""
    period_ns = int(round(1e9 / rate_hz))
    n = int(round(duration_s * rate_hz))
    sleeper = SLEEPERS[sleep_kind]
    rec = recorder or Recorder(n + 16)
    base = now_ns()
    for i in range(n):
        sleeper(base + (i + 1) * period_ns)
        rec.record(now_ns())
        if body is not None:
            body(i)
    return rec, base, period_ns, n


def small_compute(iters):
    """Return a closure that runs a fixed polynomial computation without allocating.

    Variant b uses the same computation in C++. The returned sink prevents the result
    from being optimized away. The work takes less than one loop period."""
    state = {"acc": 0.0}

    def body(i):
        x = 1.0000001
        acc = 0.0
        for _ in range(iters):
            acc = acc * x + 0.5
        state["acc"] += acc
        return acc
    return body, state
