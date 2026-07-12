#!/usr/bin/env python3
"""jitter_bench.harness -- the reusable core of the low-jitter control experiment.

One fixed-rate loop primitive, one stats primitive, and the real-time knobs
(``mlockall`` / CPU affinity / scheduling policy), all with **negligible per-cycle
overhead** and **zero non-numpy dependencies**. Everything above this module (the
variants, the matrix runner, the report) is glue.

Clock discipline (why one clock, used everywhere)
--------------------------------------------------
CPython's ``time.perf_counter``/``perf_counter_ns`` is ``clock_gettime(CLOCK_MONOTONIC)``
on Linux (verified: ``time.get_clock_info('perf_counter')``). We therefore take *every*
timestamp with ``time.clock_gettime_ns(CLOCK_MONOTONIC)`` and program *every* sleep
deadline with ``clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`` -- the sleep
deadline and the measured wake time are then points on the **same** clock, so the wakeup
latency ``wake - programmed_deadline`` is exact (no cross-clock skew). This is also the
clock ``cyclictest`` uses by default, so the Stage-1 cyclictest reference is directly
comparable to variant a1's latency histogram.

The primary metric: wakeup latency
-----------------------------------
For a fixed-rate loop we program absolute wake deadlines ``base + i*period`` and record
the actual wake time. The **wakeup latency** ``wake[i] - deadline[i]`` (how late, in µs,
the loop woke relative to the ideal fixed grid) is the cyclictest-equivalent RT number
and the one we headline. We also report the **period jitter** (consecutive interval minus
target) as a secondary diagnostic, because that is what control_kit's own bench (REPORT
§4) reported -- the two answer different questions and we give both.

No privilege is required for anything here: ``clock_nanosleep``, ``mlockall`` (fits under
the ~8 GB memlock ulimit on this box -- measured to succeed), ``sched_setaffinity`` and
``nice`` all work as an unprivileged user. ``SCHED_FIFO`` needs an rtprio grant (``ulimit
-r`` is 0 here), so ``apply_scheduling('fifo')`` *attempts* it and records the denial
rather than aborting -- the same call becomes a real FIFO run once the owner grants
rtprio (Stage 1 of the report).
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
    """The one clock: ``CLOCK_MONOTONIC`` in ns (== CPython's perf_counter clock)."""
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def clock_nanosleep_abs(deadline_ns):
    """Sleep until the absolute ``CLOCK_MONOTONIC`` deadline (ns). No privilege needed.
    Returns the libc return code (0 == slept to the deadline; nonzero == error/EINTR)."""
    ts = _timespec(deadline_ns // 1_000_000_000, deadline_ns % 1_000_000_000)
    return _LIBC.clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ctypes.byref(ts), None)


def sleep_until_time_sleep(deadline_ns):
    """The naive contrast: sleep the *remaining* time to ``deadline_ns`` with
    ``time.sleep`` (deadline-corrected, so it does not drift -- it isolates the sleep
    *mechanism* vs ``clock_nanosleep``). This is exactly control_kit's bench scheduling."""
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
    """Lock current + future pages to RAM (no paging jitter). Returns ``(ok, detail)``.
    Unprivileged on this box (memlock ulimit ~8 GB); a small process fits."""
    rc = _LIBC.mlockall(MCL_CURRENT | MCL_FUTURE)
    if rc == 0:
        return True, "mlockall(MCL_CURRENT|MCL_FUTURE) ok"
    return False, "mlockall failed (errno %d: %s)" % (
        ctypes.get_errno(), os.strerror(ctypes.get_errno()))


def try_munlockall():
    _LIBC.munlockall()


def apply_affinity(cpu):
    """Pin this process to a single CPU. Returns ``(ok, detail)``. Unprivileged."""
    if cpu is None:
        return False, "affinity not set (running on all %d cpus)" % len(os.sched_getaffinity(0))
    try:
        os.sched_setaffinity(0, {int(cpu)})
        return True, "pinned to cpu %d" % int(cpu)
    except (OSError, ValueError) as exc:
        return False, "sched_setaffinity(cpu=%r) failed: %s" % (cpu, exc)


def apply_scheduling(policy, priority=80):
    """Set the scheduling policy: ``'other'`` (default CFS, always works), ``'fifo'`` or
    ``'rr'`` (real-time, need an rtprio grant). Returns ``(ok, detail)``.

    ``SCHED_FIFO``/``SCHED_RR`` are *attempted* and the ``PermissionError`` (``ulimit -r``
    == 0 on this box) is caught and reported -- so the harness carries the exact code path
    the owner unlocks by granting rtprio, and a Stage-1 re-run is the same command with
    ``--sched fifo``. ``nice`` is also lowered best-effort under SCHED_OTHER."""
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
    """This thread's current timer slack (ns). Linux default is 50000 (50 µs)."""
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
    """A preallocated ``int64`` wake-timestamp buffer. Python variants call ``record()``
    (one indexed store, no allocation); the C++ variant writes ``.buf`` in place through
    its raw address. Sized to capture the full run; if a run overruns the capacity it
    wraps (ring) and ``wrapped`` is set -- fixed-duration runs never wrap, so full-run
    stats are honest (no window cherry-picking)."""

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
        """The recorded wake timestamps in order (full capture for a non-wrapping run)."""
        if self.wrapped:
            return self.buf.copy()
        return self.buf[:self.count].copy()


# --- stats -----------------------------------------------------------------
_PCTS = (0, 50, 90, 99, 99.9, 100)


def compute_stats(timestamps_ns, base_ns, period_ns, drop_warmup=0):
    """Turn a run's wake timestamps into the jitter stats.

    ``base_ns`` is the loop's start reference (deadline[i] == base + (i+1)*period).
    Returns a dict with wakeup-**latency** stats (µs, the headline) and period-**jitter**
    stats (µs, secondary), plus late-cycle counts and the raw arrays for histogramming.
    ``drop_warmup`` discards the first N cycles (first-use JIT / cache warmup) from the
    stats -- reported separately so the drop is explicit, never silent."""
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
    """A zero-dependency ASCII histogram of the wakeup latency (µs), with counts,
    per-bucket bar, and cumulative %. Negatives (woke slightly early) fold into ``<0``."""
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
    """Hold ``rate_hz`` for ``duration_s`` with absolute-deadline scheduling, recording
    each wake time. ``sleep_kind`` picks the wait mechanism (``SLEEPERS``); ``body(i)`` is
    the per-cycle work (the "control law"; None == pure timer). The wake timestamp is
    taken **immediately after the sleep returns**, before ``body`` -- so the latency
    isolates scheduling, not compute. Returns ``(recorder, base_ns, period_ns, n)``."""
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
    """A tiny, allocation-free arithmetic 'control law' stand-in (a fixed polynomial fold),
    matched by variant b's C++ body. Returns a closure ``body(i)`` and its result sink so
    it is never optimized away. Kept well under the period so jitter reflects *scheduling*,
    not compute (documented in the report)."""
    state = {"acc": 0.0}

    def body(i):
        x = 1.0000001
        acc = 0.0
        for _ in range(iters):
            acc = acc * x + 0.5
        state["acc"] += acc
        return acc
    return body, state
