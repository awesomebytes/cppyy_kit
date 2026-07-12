#!/usr/bin/env python3
"""jitter_bench.load -- a controllable background CPU load for the 'under load' condition.

Spins N busy-loop worker processes, each pinned to a specific CPU with ``taskset`` (no
privilege needed), so the measurement can run on a *different* core and we observe the
jitter a loaded machine induces through shared caches / memory bus / timer IRQs / global
scheduler decisions -- not raw core oversubscription. The core topology used is recorded
and printed into the report so the condition is reproducible.

``taskset``/``nice`` are unprivileged; this never touches system state. Workers are plain
``python -c 'while True: pass'`` children, killed on ``stop()`` / context exit.
"""
import atexit
import os
import signal
import subprocess
import sys


class BackgroundLoad:
    """N busy-loop processes pinned across ``load_cpus``. Use as a context manager or
    call ``start()``/``stop()``. Records the exact pinning for the report."""

    _BUSY = "x = 0\nwhile True:\n    x = (x + 1) & 0xffffffff\n"

    def __init__(self, n=8, load_cpus=None, avoid_cpu=None):
        total = sorted(os.sched_getaffinity(0))
        if load_cpus is None:
            # Default: the top N cpus, skipping the measurement cpu if given.
            pool = [c for c in total if c != avoid_cpu]
            load_cpus = pool[-n:] if n <= len(pool) else pool
        self.n = int(n)
        self.load_cpus = list(load_cpus)
        self.avoid_cpu = avoid_cpu
        self._procs = []

    def start(self):
        if self._procs:
            return self
        for i in range(self.n):
            cpu = self.load_cpus[i % len(self.load_cpus)] if self.load_cpus else None
            cmd = (["taskset", "-c", str(cpu)] if cpu is not None else []) + \
                  [sys.executable, "-c", self._BUSY]
            self._procs.append(subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        atexit.register(self.stop)
        return self

    def stop(self):
        for p in self._procs:
            if p.poll() is None:
                try:
                    p.send_signal(signal.SIGKILL)
                except OSError:
                    pass
        for p in self._procs:
            try:
                p.wait(timeout=2)
            except (subprocess.TimeoutExpired, OSError):
                pass
        self._procs = []

    def describe(self):
        return "%d busy-loop procs pinned to cpus %s (measurement avoids cpu %s)" % (
            self.n, self.load_cpus, self.avoid_cpu)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False
