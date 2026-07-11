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
