#!/usr/bin/env python3
"""jitter_bench.cpp_loop -- variant (b): the fixed-rate wait+compute loop **in C++**,
driven from Python through cppyy_kit (compile-cached + GIL-released).

The whole hot path -- ``clock_nanosleep(TIMER_ABSTIME)`` + a small compute + timestamp
record -- runs in one C++ function. Python calls it exactly once. Two cppyy_kit patterns
carry it:

* **``cppdef_cached`` (COMMON_PATTERNS §23)**: the loop body is compiled once into a
  cached ``.so`` and ``load_library``'d thereafter, so there is **no first-use call-wrapper
  JIT** stalling cycle 0 (the ~0.4-0.7 s trampoline JIT that a bare ``cppdef`` pays every
  process). The loop enters C++ at full speed on the first run after the machine's first
  build.
* **``nogil`` (COMMON_PATTERNS §27)**: the loop is invoked through the GIL-releasing shim,
  so while C++ owns the loop the interpreter is free -- a Python monitor/telemetry thread
  runs concurrently, and no GIL churn or interpreter bookkeeping sits between a wake and
  the next sleep. (For a single-threaded loop the wakeup jitter is the same with or without
  nogil; nogil is what makes a *concurrent* Python thread non-disruptive -- the honest
  reading is in the report.)

Timestamps are written straight into a caller-owned NumPy ``int64`` buffer by raw address
(the §6 "pass raw addresses" pattern), so there is **zero Python involvement per cycle** --
the point of the acceleration. Everything is ``CLOCK_MONOTONIC``, matching the Python
variants and ``clock_nanosleep``.
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
    """Compile-cache the C++ loop (§23) so the first run pays no call-wrapper JIT.
    Idempotent (cppdef_cached is one-cppdef-per-process). The nogil shim's own cached
    ``.so`` (§27) is built on the first ``nogil()`` call and reused thereafter. Returns
    the cppdef_cached result dict for the loop (``{"cached": bool, "so": ...}``)."""
    return cppyy_kit.cppdef_cached(_CODE, decls=_DECLS, name="jitter_cpp_loop")


def run_cpp_loop(rate_hz, duration_s, compute_iters=50, use_nogil=True):
    """Run the C++ fixed-rate loop for ``duration_s`` at ``rate_hz`` and return
    ``(recorder, base_ns, period_ns, n)`` -- the same shape the Python driver returns, so
    the shared ``compute_stats`` reads it identically. ``use_nogil`` invokes the loop
    through the GIL-releasing shim (the documented pattern); False calls it directly."""
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
