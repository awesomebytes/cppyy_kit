#!/usr/bin/env python3
"""Tests for the @cpp decorator (cppyy_kit.cpp).

Needs cppyy + a compiler (both in the default env), so runs under `pixi run test`.
Each test's C++ body is unique, so the per-function hashed symbol names don't clash
in the shared interpreter. Verbatim C++ type-string annotations (e.g. "float*") are
forward-ref false positives to pyflakes -- hence the `# noqa: F722,F821` (the same
convention as callback signatures, COMMON_PATTERNS §3)."""
import os
import threading
import time

import numpy as np
import pytest

import cppyy_kit
from cppyy_kit import cpp

try:
    from cppyy_kit import _compile
    _compile.cppyy_toolchain()
    _HAVE = True
except Exception:
    _HAVE = False

pytestmark = pytest.mark.skipif(not _HAVE, reason="no cppyy toolchain in this env")


def test_scalar_function():
    @cpp
    def add_i(a: int, b: int) -> int:
        """return a + b;"""
    assert int(add_i(20, 22)) == 42


def test_verbatim_scalar_type_and_double_return():
    @cpp
    def scale1(x: "double", k: "double") -> float:  # noqa: F722,F821
        """return x * k;"""
    assert abs(float(scale1(2.5, 4.0)) - 10.0) < 1e-9


def test_array_pointer_plus_size():
    @cpp
    def sum_sq(data: cpp.arr("float")) -> float:
        """double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"""
    arr = np.array([1, 2, 3, 4], dtype=np.float32)
    assert abs(float(sum_sq(arr)) - 30.0) < 1e-4


def test_pointer_mutates_in_place():
    @cpp
    def scale_inplace(y: "float*", n: int, a: float) -> None:  # noqa: F722
        """for (std::size_t i = 0; i < (std::size_t)n; ++i) y[i] *= a;"""
    arr = np.array([1, 2, 3], dtype=np.float32)
    scale_inplace(arr, arr.size, 3.0)
    assert np.allclose(arr, [3, 6, 9])


def test_body_never_executed_as_python():
    # The Python body would be a NameError if executed; @cpp uses only the docstring.
    @cpp
    def cube(x: int) -> int:
        """return x * x * x;"""
        this_is_not_python  # noqa: F821  (never runs)
    assert int(cube(3)) == 27


def test_compiles_once_reused():
    @cpp
    def inc(x: int) -> int:
        """return x + 1;"""
    assert int(inc(1)) == 2
    impl = inc._impl
    assert int(inc(41)) == 42
    assert inc._impl is impl        # not rebuilt


def test_unannotated_parameter_raises():
    with pytest.raises(TypeError):
        @cpp
        def bad(x) -> int:
            """return x;"""


def test_missing_docstring_raises():
    with pytest.raises(ValueError):
        @cpp
        def nobody(x: int) -> int:
            return x        # a real Python body, but no docstring => no C++ body


def test_exported_at_top_level():
    assert cppyy_kit.cpp is cpp


# --- @cpp(nogil=True): GIL released around only the compiled body ----------
def test_nogil_scalar_return():
    @cpp(nogil=True)
    def add_ng(a: int, b: int) -> int:
        """return a + b;"""
    assert int(add_ng(20, 22)) == 42


def test_nogil_void_mutates_in_place():
    @cpp(nogil=True)
    def scale_ng(y: "float*", n: int, a: float) -> None:  # noqa: F722
        """for (std::size_t i = 0; i < (std::size_t)n; ++i) y[i] *= a;"""
    arr = np.array([1, 2, 3], dtype=np.float32)
    scale_ng(arr, arr.size, 3.0)
    assert np.allclose(arr, [3, 6, 9])


def test_nogil_array_pointer_plus_size():
    @cpp(nogil=True)
    def sum_sq_ng(data: cpp.arr("double")) -> float:  # noqa: F821
        """double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"""
    arr = np.array([1, 2, 3, 4], dtype=np.float64)
    assert abs(float(sum_sq_ng(arr)) - 30.0) < 1e-9


def test_nogil_releases_gil_for_concurrent_thread():
    """Behavioural proof: a CPU-bound @cpp(nogil=True) kernel lets a co-thread run
    during its body, where the plain @cpp kernel (GIL held) starves it."""
    @cpp(nogil=True)
    def spin_ng(iters: int) -> None:
        """volatile double s = 0;
        for (std::size_t k = 1; k <= (std::size_t)iters; ++k) s += 1.0/(double(k)*1e-3+1.0);"""

    @cpp
    def spin_gil(iters: int) -> None:
        """volatile double s = 0;
        for (std::size_t k = 1; k <= (std::size_t)iters; ++k) s += 1.0/(double(k)*1e-3+1.0);"""

    spin_ng(1000)               # compile both before timing
    spin_gil(1000)
    t = time.perf_counter()
    spin_gil(40_000_000)
    iters = int(40_000_000 * 0.3 / max(time.perf_counter() - t, 1e-6))  # ~300 ms of work

    def co_ticks(call):
        state = {"n": 0, "stop": False}

        def worker():
            while not state["stop"]:
                state["n"] += 1
                time.sleep(0.001)

        th = threading.Thread(target=worker)
        th.start()
        time.sleep(0.03)
        start = state["n"]
        call()
        advanced = state["n"] - start
        state["stop"] = True
        th.join()
        return advanced

    held = co_ticks(lambda: spin_gil(iters))
    freed = co_ticks(lambda: spin_ng(iters))
    assert held < 25, "expected a starved co-thread with the GIL held, got %d ticks" % held
    assert freed > 100, "expected the co-thread to run under nogil, got %d ticks" % freed
    assert freed > held * 5


def test_nogil_first_use_thread_safe_single_compile():
    """N threads first-using the SAME @cpp(nogil=True) kernel at once must compile it
    exactly once (double-checked lock in _CppFunc._ensure), not once per thread -- a
    race that would re-run cppdef and make Cling emit a redefinition error."""
    from cppyy_kit import cache

    kname = "ck_cpp_mt_%d" % os.getpid()

    @cpp(nogil=True, name=kname)
    def writer(out: "double*", slot: int) -> None:  # noqa: F722
        """out[slot] = double(slot) + 1.0;"""

    calls = {"n": 0}
    real = cache.cppdef_cached

    def counting(*a, **k):
        if k.get("name") == "cpp_" + kname:
            calls["n"] += 1
        return real(*a, **k)

    n = 8
    out = np.zeros(n)
    barrier = threading.Barrier(n)

    def worker(i):
        barrier.wait()                       # release all threads into _ensure() together
        writer(out, i)

    cache.cppdef_cached = counting
    try:
        threads = [threading.Thread(target=worker, args=(i,)) for i in range(n)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    finally:
        cache.cppdef_cached = real

    assert calls["n"] == 1, "kernel compiled %d times, expected exactly 1" % calls["n"]
    assert list(out) == [i + 1 for i in range(n)], "wrong results: %r" % (list(out),)


def test_cached_false_skips_so_cache(tmp_path, monkeypatch):
    # @cpp(cached=False) compiles in-memory and writes no .so (debugging escape hatch).
    monkeypatch.setenv("CPPYY_KIT_CACHE_DIR", str(tmp_path))

    @cpp(cached=False, name="ck_nocache_%d" % os.getpid())
    def add_nc(a: int, b: int) -> int:
        """return a + b;"""
    assert int(add_nc(2, 3)) == 5

    written = []
    for root, _dirs, files in os.walk(str(tmp_path)):
        written += [f for f in files if f.endswith(".so")]
    assert written == [], "cached=False must not write a .so, found %r" % written
