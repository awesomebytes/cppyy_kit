#!/usr/bin/env python3
"""Variant b: run the fixed-rate wait and compute loop in C++ through cppyy.

The C++ function sleeps until each absolute `CLOCK_MONOTONIC` deadline, performs a
small computation, and writes the wake time to a NumPy `int64` buffer. Python calls
the function once per run. `cppdef_cached` avoids compiling the call wrapper on each
run. The `nogil` shim releases the GIL while the function runs, so other Python
threads can run. See the report for single-thread and concurrent-thread results.
"""
import cppyy

import cppyy_kit

from .harness import Recorder

# Definitions (compiled to the cached .so) and their bodiless declarations (cheap to
# cppdef on a cache hit) -- the split cppdef_cached needs to cache the glue (§23).
_CODE = r"""
#include <cstdint>
#include <cstddef>
#include <ctime>
namespace jitter_cpp {
static std::int64_t*  g_out = nullptr;
static std::size_t    g_n = 0;
static long           g_period_ns = 0;
static long           g_compute_iters = 0;
static std::int64_t   g_base_ns = 0;
static volatile double g_sink = 0.0;

static inline std::int64_t mono_ns() {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (std::int64_t)t.tv_sec * 1000000000LL + (std::int64_t)t.tv_nsec;
}

void jitter_configure(std::uintptr_t out_addr, std::size_t n,
                      long period_ns, long compute_iters) {
  g_out = reinterpret_cast<std::int64_t*>(out_addr);
  g_n = n;
  g_period_ns = period_ns;
  g_compute_iters = compute_iters;
}

std::int64_t jitter_base_ns() { return g_base_ns; }

void jitter_run() {
  const std::int64_t base = mono_ns();
  g_base_ns = base;
  double acc_total = 0.0;
  for (std::size_t i = 0; i < g_n; ++i) {
    std::int64_t deadline = base + (std::int64_t)(i + 1) * g_period_ns;
    struct timespec ts;
    ts.tv_sec  = deadline / 1000000000LL;
    ts.tv_nsec = deadline % 1000000000LL;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    g_out[i] = mono_ns();
    // small 'control law': the same polynomial fold the Python body runs
    double x = 1.0000001, acc = 0.0;
    for (long k = 0; k < g_compute_iters; ++k) acc = acc * x + 0.5;
    acc_total += acc;
  }
  g_sink = acc_total;
}
}  // namespace jitter_cpp
"""

_DECLS = r"""
#include <cstdint>
#include <cstddef>
namespace jitter_cpp {
void jitter_configure(std::uintptr_t out_addr, std::size_t n, long period_ns, long compute_iters);
std::int64_t jitter_base_ns();
void jitter_run();
}
"""

def ensure_built():
    """Compile-cache the C++ loop so later runs do not JIT its call wrapper.
    The operation is idempotent within a process. The nogil shim builds and caches its
    own shared library on its first call. Returns the cppdef_cached result dictionary."""
    return cppyy_kit.cppdef_cached(_CODE, decls=_DECLS, name="jitter_cpp_loop")


def run_cpp_loop(rate_hz, duration_s, compute_iters=50, use_nogil=True):
    """Run the C++ loop for ``duration_s`` at ``rate_hz``.

    Returns ``(recorder, base_ns, period_ns, n)``, which is also returned by the Python
    driver and accepted by ``compute_stats``. If ``use_nogil`` is true, call through the
    GIL-releasing shim. Otherwise, call the function directly."""
    period_ns = int(round(1e9 / rate_hz))
    n = int(round(duration_s * rate_hz))
    ensure_built()
    rec = Recorder(n)
    rec.count = n                      # C++ fills every slot in place; mark it full
    jc = cppyy.gbl.jitter_cpp
    jc.jitter_configure(int(rec.buf.ctypes.data), n, period_ns, int(compute_iters))
    if use_nogil:
        cppyy_kit.nogil(jc.jitter_run)          # GIL released for the whole loop (§27)
    else:
        jc.jitter_run()
    base_ns = int(jc.jitter_base_ns())
    return rec, base_ns, period_ns, n
