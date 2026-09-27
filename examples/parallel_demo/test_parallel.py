#!/usr/bin/env python3
"""Check output equivalence for the parallelism example.

The example compares independent C++ jobs called from Python threads with the GIL
held and released. This test checks that both modes produce the same values; runtime
speedup depends on the host and is demonstrated by the runnable example. Requires
cppyy and a compiler (available in the default environment)."""
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
