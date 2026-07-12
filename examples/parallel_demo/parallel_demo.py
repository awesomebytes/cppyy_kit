#!/usr/bin/env python3
"""True multi-core parallelism from plain Python threads, via cppyy_kit.nogil.

Python's GIL serialises threads, so eight pure-Python threads doing CPU work take as
long as one. cppyy_kit.nogil() releases the GIL around a C++ call, so N Python
threads each driving a C++ kernel run genuinely in parallel on N cores.

There is no trick: nogil() wraps the C++ callable in Py_BEGIN_ALLOW_THREADS /
Py_END_ALLOW_THREADS, dropping the interpreter lock for the duration of the C++ work
and re-taking it after (see cppyy_kit/nogil.py and COMMON_PATTERNS section 13). The
jobs must be independent and write their results into C++ memory -- here, disjoint
slots of a NumPy array -- so no thread needs the GIL while it computes.

Run:  python examples/parallel_demo/parallel_demo.py
Needs cppyy + a C++ compiler (the default env).
"""
import os
import threading
import time

import numpy as np

import cppyy
import cppyy_kit

# A CPU-bound C++ kernel, plus a factory that returns a nullary std::function<void()>
# bound to one output slot. cppdef_cached compiles it once and caches the .so.
_SRC = r"""
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <functional>
namespace ck_parallel {
  void crunch(double* out, std::size_t slot, std::size_t iters) {
    double s = 0.0;
    for (std::size_t k = 1; k <= iters; ++k) s += std::sin(double(k) * 1e-6);
    out[slot] = s;
  }
  std::function<void()> task(std::uintptr_t out_addr, std::size_t slot, std::size_t iters) {
    double* out = reinterpret_cast<double*>(out_addr);
    return [=] { crunch(out, slot, iters); };
  }
}
"""
# Bodiless declarations let cppdef_cached compile the kernel to a real .so and cache
# it (dlopen thereafter), instead of re-parsing the body every run.
_DECLS = r"""
#include <cstdint>
#include <cstddef>
#include <functional>
namespace ck_parallel {
  void crunch(double* out, std::size_t slot, std::size_t iters);
  std::function<void()> task(std::uintptr_t out_addr, std::size_t slot, std::size_t iters);
}
"""
cppyy_kit.cppdef_cached(_SRC, decls=_DECLS, name="parallel_demo_kernel")


def _tasks(out, iters):
    """One nullary C++ callable per output slot (each writes out[slot])."""
    return [cppyy.gbl.ck_parallel.task(out.ctypes.data, i, iters)
            for i in range(len(out))]


def warm():
    """Compile the nogil shim + kernel once, single-threaded, before any thread uses
    them. nogil()'s first-use compile is not thread-safe, so calling it first from
    several threads at once races; a single-threaded warm-up avoids that."""
    out = np.zeros(1)
    cppyy_kit.nogil(cppyy.gbl.ck_parallel.task(out.ctypes.data, 0, 1000))


def run(n_threads, iters, use_nogil):
    """Run n_threads independent C++ jobs on Python threads; return (wall_s, out).

    use_nogil=True releases the GIL around each C++ call (true parallelism).
    use_nogil=False calls the C++ std::function directly -- cppyy holds the GIL for
    the call, so the threads serialise, which is the plain-Python behaviour."""
    out = np.zeros(n_threads)
    tasks = _tasks(out, iters)

    def call(t):
        cppyy_kit.nogil(t) if use_nogil else t()

    threads = [threading.Thread(target=call, args=(t,)) for t in tasks]
    t0 = time.perf_counter()
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    return time.perf_counter() - t0, out


def main():
    n = 8
    iters = 20_000_000
    warm()                                           # compile shim + kernel once

    serial, a = run(n, iters, use_nogil=False)
    parallel, b = run(n, iters, use_nogil=True)
    assert np.allclose(a, b), "results diverged"     # identical output either way

    print("cores available:            %d" % (os.cpu_count() or 1))
    print("%d C++ jobs, GIL held:      %8.1f ms" % (n, serial * 1e3))
    print("%d C++ jobs, GIL released:  %8.1f ms   (nogil)" % (n, parallel * 1e3))
    print("speedup:                    %.1fx" % (serial / parallel))


if __name__ == "__main__":
    main()
