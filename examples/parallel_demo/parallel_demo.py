#!/usr/bin/env python3
"""True multi-core parallelism from plain Python threads, via @cpp(nogil=True).

Python's GIL serialises threads, so eight pure-Python threads doing CPU work take as
long as one. A kernel written with @cpp(nogil=True) releases the GIL around its
compiled body, so N Python threads each calling the kernel run genuinely in parallel
on N cores.

There is no trick: @cpp(nogil=True) wraps the compiled body in Py_BEGIN_ALLOW_THREADS
/ Py_END_ALLOW_THREADS, dropping the interpreter lock for the duration of the C++ work
and re-taking it after -- while cppyy's argument/result marshaling stays under the lock
(see cppyy_kit/_cpp.py and COMMON_PATTERNS section 13/27). The jobs must be independent
and write their results into C++ memory -- here, disjoint slots of a NumPy array -- so
no thread needs the GIL while it computes.

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
# The kernel is header-free arithmetic (a reciprocal sum) so it needs no includes
# beyond @cpp's own -- and is unambiguously CPU-bound. (The body is the docstring, so
# it must be an inline literal -- hence the two identical bodies below.)
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
    """Compile both kernels once before the timed runs, so those runs measure the
    parallel work rather than one-time setup. (@cpp's first-use compile is thread-safe,
    so this warm-up is for timing accuracy, not correctness -- threads may safely take
    the first-use path concurrently.)"""
    out = np.zeros(1)
    crunch_parallel(out, 0, 1000)
    crunch_gil(out, 0, 1000)


def run(n_threads, iters, use_nogil):
    """Run n_threads independent C++ jobs on Python threads; return (wall_s, out).

    use_nogil=True calls the @cpp(nogil=True) kernel, which releases the GIL around its
    body (true parallelism). use_nogil=False calls the plain @cpp kernel -- cppyy holds
    the GIL for the call, so the threads serialise, which is the plain-Python behaviour.
    Each thread writes a disjoint output slot, so results are identical either way."""
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
