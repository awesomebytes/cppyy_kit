"""jitter_bench -- the M6g low-jitter Python control experiment.

Measures, honestly, the loop-period jitter of a ~1 kHz control loop orchestrated from
Python, on the machine's current kernel (6.17 oem, PREEMPT_DYNAMIC -- NOT PREEMPT_RT).
Three loop bodies through one harness:

  * ``a1`` -- pure-Python timer loop, ``clock_nanosleep(TIMER_ABSTIME)`` absolute sleeps.
  * ``a2`` -- pure-Python timer loop, deadline-corrected ``time.sleep`` (the naive contrast).
  * ``b``  -- the wait+compute loop **in C++** via cppyy_kit (``cppdef_cached`` + ``nogil``).
  * ``c``  -- the real in-process ros2_control update loop, driven from Python (control_kit).

Each variant runs idle and under background CPU load; ``mlockall`` / CPU pinning /
scheduling policy are applied best-effort and their outcomes recorded. The primary metric
is wakeup latency vs the programmed absolute deadline (the cyclictest-equivalent).

Reference numbers only, on the current kernel; the SCHED_FIFO + preemption-mode comparison
is a later phase gated on owner actions (see docs/jitter_bench/REPORT.md, Stage 1).

Entry point: ``python jitter_bench/run_bench.py`` (see ``--help``).
"""
