#!/usr/bin/env python3
"""Tests for cppyy_kit.nogil -- the GIL-release shim (COMMON_PATTERNS §13).

The proof is behavioural: a blocking C++ call made directly holds the GIL and starves
a concurrent Python thread; through nogil() the GIL is released and the thread runs.
Needs cppyy + a compiler (default env)."""
import asyncio
import threading
import time

import pytest

import cppyy

from cppyy_kit import nogil, run_async

try:
    from cppyy_kit import _compile
    _compile.cppyy_toolchain()
    _HAVE = True
except Exception:
    _HAVE = False

pytestmark = pytest.mark.skipif(not _HAVE, reason="no cppyy toolchain in this env")

if _HAVE:
    cppyy.cppdef(r"""
    #include <thread>
    #include <chrono>
    namespace ck_nogil_test {
      void sleep_300() { std::this_thread::sleep_for(std::chrono::milliseconds(300)); }
    }
    """)


def _co_thread_ticks(blocking_call):
    """Ticks a background Python thread accrues *during* blocking_call()."""
    state = {"n": 0, "stop": False}

    def worker():
        while not state["stop"]:
            state["n"] += 1
            time.sleep(0.001)

    t = threading.Thread(target=worker)
    t.start()
    time.sleep(0.03)                 # let the worker get going
    start = state["n"]
    blocking_call()                  # ~300 ms of C++ sleep
    advanced = state["n"] - start
    state["stop"] = True
    t.join()
    return advanced


def test_nogil_releases_gil_for_concurrent_thread():
    direct = _co_thread_ticks(lambda: cppyy.gbl.ck_nogil_test.sleep_300())
    freed = _co_thread_ticks(lambda: nogil(cppyy.gbl.ck_nogil_test.sleep_300))
    # GIL held across the direct C++ call -> co-thread starved; released via nogil -> runs.
    assert direct < 25, "expected a starved co-thread, got %d ticks" % direct
    assert freed > 100, "expected the co-thread to run, got %d ticks" % freed
    assert freed > direct * 5


def test_run_async_lets_event_loop_run():
    async def main():
        ticks = {"n": 0}

        async def ticker():
            try:
                while True:
                    ticks["n"] += 1
                    await asyncio.sleep(0.001)
            except asyncio.CancelledError:
                pass

        task = asyncio.ensure_future(ticker())
        await run_async(cppyy.gbl.ck_nogil_test.sleep_300)   # blocking C++, GIL released
        task.cancel()
        await task
        return ticks["n"]

    n = asyncio.run(main())
    assert n > 100, "event loop should keep running during the blocking C++ call, got %d" % n


def test_ensure_is_thread_safe_single_compile():
    """Regression: the first-ever nogil() calls arriving from many threads at once must
    compile the shim exactly once (double-checked lock in _ensure), not once per thread
    (which raced and made Cling emit a "redefinition of run_nogil" error)."""
    import importlib

    import numpy as np

    nogil_mod = importlib.import_module("cppyy_kit.nogil")   # the module (cppyy_kit.nogil is the fn)

    cppyy.cppdef(r"""
    #include <cstdint>
    #include <cstddef>
    #include <functional>
    namespace ck_nogil_mt {
      std::function<void()> setter(std::uintptr_t out, std::size_t i) {
        auto* p = reinterpret_cast<long*>(out);
        return [=] { p[i] = long(i) + 1; };
      }
    }
    """)

    calls = {"n": 0}
    real = nogil_mod.cache.cppdef_cached

    def counting(*a, **k):
        if k.get("name") == "nogil_shim":
            calls["n"] += 1
        return real(*a, **k)

    n = 8
    out = np.zeros(n, dtype=np.int64)
    barrier = threading.Barrier(n)

    def worker(i):
        barrier.wait()                       # release all threads into _ensure() together
        nogil_mod.nogil(cppyy.gbl.ck_nogil_mt.setter(out.ctypes.data, i))

    nogil_mod._READY = False                 # force a fresh first-use
    nogil_mod.cache.cppdef_cached = counting
    try:
        threads = [threading.Thread(target=worker, args=(i,)) for i in range(n)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    finally:
        nogil_mod.cache.cppdef_cached = real
        nogil_mod._READY = True              # shim is compiled now

    assert calls["n"] == 1, "shim compiled %d times, expected exactly 1" % calls["n"]
    assert list(out) == [i + 1 for i in range(n)], "wrong results: %r" % (list(out),)
