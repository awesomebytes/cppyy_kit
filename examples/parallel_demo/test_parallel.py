#!/usr/bin/env python3
"""Parallelism contract for the multithreading example: N Python threads each calling
a @cpp(nogil=True) kernel run on N cores, so N jobs finish far faster than the same
jobs with the GIL held -- and produce identical output either way.

@cpp(nogil=True) releases the GIL around the compiled body (COMMON_PATTERNS section
13/27); the jobs are independent and write into disjoint NumPy slots, so no thread
needs the GIL while computing. Needs cppyy + a compiler (the default env)."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))

try:
    from cppyy_kit import _compile
    _compile.cppyy_toolchain()
    _HAVE = True
except Exception:
    _HAVE = False

pytestmark = pytest.mark.skipif(not _HAVE, reason="no cppyy toolchain in this env")

_CORES = os.cpu_count() or 1


def _run(**kw):
    from parallel_demo import run
    return run(**kw)


def _warm():
    from parallel_demo import warm
    warm()


def test_results_identical_gil_held_vs_released():
    _warm()
    _, held = _run(n_threads=4, iters=200_000, use_nogil=False)
    _, freed = _run(n_threads=4, iters=200_000, use_nogil=True)
    assert np.allclose(held, freed)


@pytest.mark.skipif(_CORES < 4, reason="needs >=4 cores to show parallelism")
def test_nogil_threads_run_in_parallel():
    n = 8
    iters = 20_000_000
    _warm()                                                      # single-threaded init
    serial, _ = _run(n_threads=n, iters=iters, use_nogil=False)
    parallel, _ = _run(n_threads=n, iters=iters, use_nogil=True)
    speedup = serial / parallel
    # GIL held serializes the threads (speedup ~1x); released, the ceiling is
    # min(n, cores). 45% of that ceiling passes on a quiet 16-core box (~7.7x
    # measured) and on a 4-vCPU shared CI runner (2.3x measured), while a
    # GIL-bound run (~1x) always fails.
    floor = 0.45 * min(n, _CORES)
    assert speedup > floor, \
        "expected >%.1fx from nogil parallelism on %d cores, got %.1fx" % (
            floor, _CORES, speedup)
