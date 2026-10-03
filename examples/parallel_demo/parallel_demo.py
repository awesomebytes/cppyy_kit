#!/usr/bin/env python3
"""Demonstrate C++ work from Python threads with @cpp(nogil=True).

Python's GIL limits concurrent execution of Python bytecode. A kernel written with
@cpp(nogil=True) releases the GIL while its compiled body runs, allowing independent
jobs to overlap. Actual speedup depends on available CPU resources and system load.

Argument and result marshaling occurs with the GIL held. The example uses independent
jobs that write to disjoint NumPy array slots, so each compiled computation can run
without accessing Python objects.

Run:  python examples/parallel_demo/parallel_demo.py
Needs cppyy + a C++ compiler (the default env).
"""
import os
import threading
import time

import numpy as np

from cppyy_kit import cpp

# The same CPU-bound kernel, once with the GIL released and once with it held. @cpp
# compiles each into a cached .so on first call and marshals the NumPy array (as
# out.ctypes.data) into the `double*` parameter for us; `slot` picks the output slot.
# The kernel computes a reciprocal sum and needs no includes. Each kernel body is
# an inline string, so both functions contain the same body.
@cpp(nogil=True)
def crunch_parallel(out: "double*", slot: int, iters: int) -> None:  # noqa: F722,F821
    """double s = 0.0;
    for (std::size_t k = 1; k <= (std::size_t)iters; ++k) s += 1.0 / (double(k) * 1e-3 + 1.0);
    out[slot] = s;"""


@cpp
def crunch_gil(out: "double*", slot: int, iters: int) -> None:  # noqa: F722,F821
    """double s = 0.0;
    for (std::size_t k = 1; k <= (std::size_t)iters; ++k) s += 1.0 / (double(k) * 1e-3 + 1.0);
    out[slot] = s;"""


def warm():
    """Compile both kernels before timing so the runs measure their work, not setup.
    First-use compilation is thread-safe, so this warm-up is for timing accuracy."""
    out = np.zeros(1)
    crunch_parallel(out, 0, 1000)
    crunch_gil(out, 0, 1000)


def run(n_threads, iters, use_nogil):
    """Run ``n_threads`` independent C++ jobs from Python threads. Return wall time
    and the output array. With ``use_nogil=True``, the kernel releases the GIL and
    threads can run in parallel. Otherwise cppyy holds the GIL during each call, so
    the threads run one at a time. Each thread writes to a separate output slot."""
    out = np.zeros(n_threads)
    kernel = crunch_parallel if use_nogil else crunch_gil

    threads = [threading.Thread(target=kernel, args=(out, i, iters))
               for i in range(n_threads)]
    t0 = time.perf_counter()
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    return time.perf_counter() - t0, out


def main():
    n = 8
    iters = 20_000_000
    warm()                                           # compile both kernels once

    serial, a = run(n, iters, use_nogil=False)
    parallel, b = run(n, iters, use_nogil=True)
    assert np.allclose(a, b), "results diverged"     # identical output either way

    print("cores available:            %d" % (os.cpu_count() or 1))
    print("%d C++ jobs, GIL held:      %8.1f ms" % (n, serial * 1e3))
    print("%d C++ jobs, GIL released:  %8.1f ms   (@cpp(nogil=True))" % (n, parallel * 1e3))
    print("speedup:                    %.1fx" % (serial / parallel))


if __name__ == "__main__":
    main()
