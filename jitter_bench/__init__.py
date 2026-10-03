"""Measure timing jitter in a Python-driven control loop near 1 kHz.

The benchmark uses the current Linux kernel (6.17 oem, PREEMPT_DYNAMIC, not
PREEMPT_RT). It compares four loop variants while idle and under CPU load. The
harness applies memory locking, CPU affinity, and scheduling settings when possible
and records whether they succeeded.

The main metric is wakeup latency against an absolute deadline. The report contains
reference numbers for the current kernel. A comparison using SCHED_FIFO and different
preemption modes requires owner system changes; see `docs/jitter_bench/REPORT.md`.

Run `python jitter_bench/run_bench.py`; see `--help` for options.
"""
