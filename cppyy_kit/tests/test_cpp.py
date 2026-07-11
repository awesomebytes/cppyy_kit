#!/usr/bin/env python3
"""Tests for the @cpp decorator (cppyy_kit.cpp).

Needs cppyy + a compiler (both in the default env), so runs under `pixi run test`.
Each test's C++ body is unique, so the per-function hashed symbol names don't clash
in the shared interpreter. Verbatim C++ type-string annotations (e.g. "float*") are
forward-ref false positives to pyflakes -- hence the `# noqa: F722,F821` (the same
convention as callback signatures, COMMON_PATTERNS §3)."""
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
