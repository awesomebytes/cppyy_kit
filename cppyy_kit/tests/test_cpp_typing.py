"""Numeric annotations and value-based specialization for ``@cpp``."""
import os
import subprocess
import sys
import threading
import typing

import numpy as np
from numpy.typing import NDArray
import pytest

from cppyy_kit import cache, cpp


def _count_compiles(monkeypatch):
    calls = []
    real = cache.cppdef_cached

    def counting(*args, **kwargs):
        calls.append(kwargs.get("name"))
        return real(*args, **kwargs)

    monkeypatch.setattr(cache, "cppdef_cached", counting)
    return calls


def _assert_no_specialization(kernel):
    assert getattr(kernel, "_impl", None) is None
    assert not getattr(kernel, "_impls", {})


@pytest.mark.parametrize("dtype", [
    np.int8, np.uint8, np.int16, np.uint16, np.int32, np.uint32,
    np.int64, np.uint64, np.float32, np.float64, np.bool_,
    np.complex64, np.complex128,
])
def test_inferred_ndarray_dtype_has_native_representation(dtype):
    @cpp(cached=False)
    def element_size(values) -> int:
        """return sizeof(values[0]);"""

    values = np.array([0, 1], dtype=dtype)
    assert int(element_size(values)) == values.dtype.itemsize


@pytest.mark.parametrize("dtype", [np.int8, np.uint8, np.int16, np.uint16,
                                   np.int32, np.uint32, np.int64, np.uint64,
                                   np.float32, np.float64, np.bool_,
                                   np.complex64, np.complex128])
def test_explicit_ndarray_dtype_has_native_representation(dtype):
    @cpp(cached=False)
    def element_size(values: np.ndarray[typing.Any, np.dtype[dtype]]) -> int:
        """return sizeof(values[0]);"""

    values = np.array([0, 1], dtype=dtype)
    assert int(element_size(values)) == values.dtype.itemsize


@pytest.mark.parametrize("annotation", [np.ndarray, NDArray])
def test_bare_array_annotations_infer_each_numeric_dtype(annotation):
    @cpp(cached=False)
    def element_size(values: annotation) -> int:
        """return sizeof(values[0]);"""

    for dtype in (np.int16, np.float32, np.complex128):
        values = np.array([0, 1], dtype=dtype)
        assert int(element_size(values)) == values.dtype.itemsize


def test_ndarray_borrowed_in_place_and_annotation_forms():
    from numpy.typing import NDArray

    @cpp(cached=False)
    def scale(a: NDArray[np.float32], factor: float) -> None:
        """for (std::size_t i = 0; i < a_size; ++i) a[i] *= factor;"""

    a = np.array([1, 2, 3], dtype=np.float32)
    scale(a, 2.0)
    assert np.array_equal(a, [2, 4, 6])


def test_missing_return_annotation_defaults_to_void():
    @cpp(cached=False)
    def write(values: np.ndarray[typing.Any, np.dtype[np.int32]]):
        """values[0] = 17;"""

    values = np.zeros(2, dtype=np.int32)
    assert write(values) is None
    assert values[0] == 17


def test_future_annotations_are_resolved():
    ns = {"cpp": cpp, "np": np}
    exec("from __future__ import annotations\n"
         "from numpy.typing import NDArray\n"
         "@cpp(cached=False)\n"
         "def first(a: NDArray[np.float32]) -> int:\n"
         "    'return sizeof(a[0]);'\n", ns)
    assert int(ns["first"](np.ones(2, dtype=np.float32))) == 4


def test_future_builtin_float_and_quoted_cpp_float_stay_distinct():
    ns = {"cpp": cpp}
    exec("from __future__ import annotations\n"
         "@cpp(cached=False)\n"
         "def builtin_size(x: float) -> int:\n"
         "    'return sizeof(x);'\n"
         "@cpp(cached=False)\n"
         "def cpp_size(x: 'float') -> int:\n"
         "    'return sizeof(x);'\n"
         "@cpp(cached=False)\n"
         "def cpp_identity(x: 'float') -> 'float':\n"
         "    'return x;'\n", ns)
    assert int(ns["builtin_size"](1.0)) == 8
    assert int(ns["cpp_size"](1.0)) == 4
    assert float(ns["cpp_identity"](1.1)) == float(np.float32(1.1))


def test_literal_cpp_float_parameter_keeps_float_representation():
    @cpp(cached=False)
    def size(x: "float") -> int:  # noqa: F722
        """return sizeof(x);"""

    assert int(size(2.5)) == 4


@pytest.mark.parametrize("annotation", [list[float], typing.List[float],
                                        tuple[float, ...], typing.Tuple[float, ...],
                                        typing.Sequence[float]])
def test_typed_sequences_are_copied_and_not_copied_back(annotation):
    @cpp(cached=False)
    def mutate(values: annotation) -> int:
        """values[0] = 99; return values_size;"""

    values = [1.0, 2.0]
    assert int(mutate(values)) == 2
    assert values == [1.0, 2.0]


def test_typed_empty_sequence_and_nogil_owned_buffer_lifetime():
    @cpp(cached=False, nogil=True)
    def total(values: list[np.float64]) -> float:
        """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    assert float(total([])) == 0.0
    assert float(total([1.5, 2.5])) == 4.0


def test_typed_numpy_float_sequence_rounds_normally_and_accepts_nonfinite():
    @cpp(cached=False)
    def total(values: list[np.float32]) -> float:
        """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    assert float(total([0.1])) == pytest.approx(float(np.float32(0.1)))
    assert np.isnan(float(total([float("nan")])))
    assert np.isinf(float(total([float("inf")])))


def test_sequence_annotations_accept_general_sequence_containers():
    @cpp(cached=False)
    def total(values: typing.Sequence[int]) -> int:
        """int s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    assert int(total(range(5))) == 10
    assert int(total((1, 2, 3))) == 6


@pytest.mark.parametrize("values", [[], [1, 2.0], ["a"], [[1, 2]]])
def test_unsupported_inferred_sequences_fail_before_compilation(values):
    @cpp(cached=False)
    def first(values):
        """return 0;"""

    with pytest.raises((TypeError, ValueError)):
        first(values)
    _assert_no_specialization(first)


@pytest.mark.parametrize("dtype", [np.float16, np.dtype(">f4"), np.dtype("U4"),
                                   np.dtype("O")])
def test_unsupported_ndarray_dtypes_fail_without_compilation(dtype):
    @cpp(cached=False)
    def first(values):
        """return 0;"""

    values = np.array([1, 2], dtype=dtype)
    with pytest.raises((TypeError, ValueError)):
        first(values)
    _assert_no_specialization(first)


def test_array_layout_alignment_and_readonly_are_rejected_without_copy():
    @cpp(cached=False)
    def first(values):
        """return 0;"""

    valid = np.arange(8, dtype=np.float32)
    bad = [valid[::2], valid[::-1], np.arange(8, dtype=np.float32)[1:].view(np.float32)]
    bad[-1].flags.aligned = False
    for values in bad:
        with pytest.raises((TypeError, ValueError)):
            first(values)
        _assert_no_specialization(first)

    readonly = valid.copy()
    readonly.flags.writeable = False
    with pytest.raises((TypeError, ValueError)):
        first(readonly)
    _assert_no_specialization(first)


def test_typed_array_mismatch_and_python_int_range_errors_precede_compile():
    @cpp(cached=False)
    def first(values: np.ndarray[typing.Any, np.dtype[np.float32]]) -> int:
        """return 0;"""

    with pytest.raises((TypeError, ValueError)):
        first(np.ones(2, dtype=np.float64))
    _assert_no_specialization(first)

    @cpp(cached=False)
    def ints(values: list[int]) -> int:
        """return values_size;"""

    with pytest.raises((TypeError, ValueError, OverflowError)):
        ints([2**100])
    _assert_no_specialization(ints)


def test_typed_sequence_rejects_invalid_values_before_compilation():
    @cpp(cached=False)
    def values_sum(values: list[float]) -> float:
        """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    with pytest.raises((TypeError, ValueError, OverflowError)):
        values_sum([1.0, "not a float"])
    _assert_no_specialization(values_sum)


@pytest.mark.parametrize("values", [[1.5], [2**100]])
def test_typed_integer_sequence_rejects_noninteger_or_out_of_range(values):
    @cpp(cached=False)
    def values_sum(values: list[int]) -> int:
        """int s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    with pytest.raises((TypeError, ValueError, OverflowError)):
        values_sum(values)
    _assert_no_specialization(values_sum)

    assert int(values_sum([True])) == 1


def test_typed_boolean_sequence_accepts_only_booleans():
    @cpp(cached=False)
    def values_sum(values: list[bool]) -> int:
        """int s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    assert int(values_sum([True, False, True])) == 2
    with pytest.raises((TypeError, ValueError)):
        values_sum([1, 0])


def test_complex_values_cannot_be_converted_to_real_sequence():
    @cpp(cached=False)
    def values_sum(values: list[float]) -> float:
        """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    with pytest.raises((TypeError, ValueError)):
        values_sum([1 + 2j])
    _assert_no_specialization(values_sum)


@pytest.mark.parametrize("value, expected_size", [(True, 1), (3, 4), (2.5, 8),
                                                   (np.int16(3), 2),
                                                   (np.float32(2), 4),
                                                   (np.complex64(1j), 8),
                                                   (1 + 2j, 16)])
def test_unannotated_scalar_numeric_inference(value, expected_size):
    @cpp(cached=False)
    def size(x) -> int:
        """return sizeof(x);"""
    assert int(size(value)) == expected_size


@pytest.mark.parametrize("scalar, value, expected_size", [
    (np.int16, 12, 2), (np.float32, 0.1, 4),
    (np.complex64, 1 + 2j, 8),
])
def test_numpy_scalar_type_annotations(scalar, value, expected_size):
    @cpp(cached=False)
    def size(x: scalar) -> int:
        """return sizeof(x);"""

    assert int(size(value)) == expected_size


def test_inferred_scalar_results_and_float32_rounding():
    @cpp(cached=False)
    def twice_plus_one(x) -> int:
        """return x * 2 + 1;"""

    @cpp(cached=False)
    def add_float(x: np.float32) -> float:
        """return x + 0.2f;"""

    @cpp(cached=False)
    def scale_complex(x) -> complex:
        """return x * std::complex<double>(2.0, -1.0);"""

    @cpp(cached=False)
    def complex_parts(x) -> float:
        """return x.real() + 10.0 * x.imag();"""

    assert int(twice_plus_one(20)) == 41
    expected = float(np.float32(np.float32(0.1) + np.float32(0.2)))
    assert float(add_float(0.1)) == expected
    assert complex(scale_complex(1 + 2j)) == complex(4 + 3j)
    assert float(complex_parts(1.5 + 2.0j)) == 21.5


def test_typed_integer_scalar_overflow_fails_before_compilation():
    @cpp(cached=False)
    def identity(x: np.int8) -> int:
        """return x;"""

    with pytest.raises((TypeError, ValueError, OverflowError)):
        identity(128)
    _assert_no_specialization(identity)
    assert int(identity(127)) == 127


def test_scalar_and_two_input_variants_are_keyed_by_types_not_values(monkeypatch):
    calls = _count_compiles(monkeypatch)

    @cpp(cached=False, name="typing_scalar_dispatch_%d" % os.getpid())
    def add(a, b) -> float:
        """return a + b;"""

    assert float(add(1.0, 2.0)) == 3.0
    assert float(add(4.0, 5.0)) == 9.0
    assert float(add(1.0, 2.0,)) == 3.0
    assert len(calls) == 1

    assert float(add(np.float32(1), np.float32(2))) == 3.0
    assert len(calls) == 2
    assert float(add(np.float32(10), np.float32(20))) == 30.0
    assert len(calls) == 2


def test_inferred_array_lengths_reuse_one_specialization(monkeypatch):
    calls = _count_compiles(monkeypatch)

    @cpp(cached=False, name="typing_array_lengths_%d" % os.getpid())
    def total(values) -> float:
        """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    for n in (0, 1, 5, 100):
        a = np.arange(n, dtype=np.float64)
        assert float(total(a)) == float(a.sum())
    assert len(calls) == 1


def test_inferred_list_tuple_and_array_share_dtype_specialization(monkeypatch):
    calls = _count_compiles(monkeypatch)

    @cpp(cached=False, name="typing_buffer_dispatch_%d" % os.getpid())
    def total(values) -> float:
        """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""

    assert float(total([1.0, 2.0])) == 3.0
    assert float(total((2.5,))) == 2.5
    assert float(total(np.array([1.25, 2.75, 3.0], dtype=np.float64))) == 7.0
    assert len(calls) == 1


def test_concurrent_first_calls_compile_one_variant(monkeypatch):
    calls = _count_compiles(monkeypatch)

    @cpp(cached=False, name="typing_concurrent_%d" % os.getpid())
    def fill(values):
        """for (std::size_t i = 0; i < values_size; ++i) values[i] = double(i + 1);"""

    n = 6
    arrays = [np.zeros(4, dtype=np.float64) for _ in range(n)]
    barrier = threading.Barrier(n)
    errors = []

    def worker(i):
        try:
            barrier.wait()
            fill(arrays[i])
        except BaseException as exc:  # make thread failures visible in the test
            errors.append(exc)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(n)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not errors
    assert len(calls) == 1
    assert all(np.array_equal(a, [1, 2, 3, 4]) for a in arrays)


def test_same_name_and_body_get_distinct_symbols_for_array_dtypes():
    @cpp(cached=False, name="typing_dtype_symbol_%d" % os.getpid())
    def float_size(values) -> int:
        """return sizeof(values[0]);"""

    @cpp(cached=False, name="typing_dtype_symbol_%d" % os.getpid())
    def double_size(values) -> int:
        """return sizeof(values[0]);"""

    assert int(float_size(np.ones(2, dtype=np.float32))) == 4
    assert int(double_size(np.ones(2, dtype=np.float64))) == 8


def test_disk_cache_reused_across_independent_processes(tmp_path):
    env = os.environ.copy()
    env["CPPYY_KIT_CACHE_DIR"] = str(tmp_path)
    script = (
        "import numpy as np\n"
        "from cppyy_kit import cpp\n"
        "@cpp(name='typing_disk_reuse', cached=True)\n"
        "def total(values) -> float:\n"
        "    'double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;'\n"
        "assert float(total(np.array([1., 2.], dtype=np.float32))) == 3.0\n"
    )
    warm_script = script.replace(
        "from cppyy_kit import cpp\n",
        "from cppyy_kit import cache, cpp\n"
        "def fail_build(*args, **kwargs):\n"
        "    raise AssertionError('warm cache attempted a cold build')\n"
        "cache._build = fail_build\n",
    )
    first = subprocess.run([sys.executable, "-c", script], env=env, text=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=90)
    second = subprocess.run([sys.executable, "-c", warm_script], env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=90)
    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    cache_files = [p for p in tmp_path.rglob("*") if p.suffix == ".so"]
    assert len(cache_files) == 1


def test_cross_wrapper_cached_false_then_true_persists_for_next_process(tmp_path, monkeypatch):
    monkeypatch.setenv("CPPYY_KIT_CACHE_DIR", str(tmp_path))

    @cpp(cached=False, name="typing_cross_cache")
    def first(values) -> float:
        """return values[0] + values[1];"""

    assert float(first(np.array([1.5, 2.5], dtype=np.float32))) == 4.0
    assert not list(tmp_path.rglob("*.so"))

    @cpp(cached=True, name="typing_cross_cache")
    def second(values) -> float:
        """return values[0] + values[1];"""

    assert float(second(np.array([1.5, 2.5], dtype=np.float32))) == 4.0
    cache_files = list(tmp_path.rglob("*.so"))
    assert len(cache_files) == 1

    script = (
        "import numpy as np\n"
        "from cppyy_kit import cache, cpp\n"
        "def fail_build(*args, **kwargs):\n"
        "    raise AssertionError('warm cache attempted a cold build')\n"
        "cache._build = fail_build\n"
        "@cpp(name='typing_cross_cache', cached=True)\n"
        "def total(values) -> float:\n"
        "    'return values[0] + values[1];'\n"
        "assert float(total(np.array([3.0, 4.0], dtype=np.float32))) == 7.0\n"
    )
    child = subprocess.run([sys.executable, "-c", script], env=os.environ.copy(),
                           text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=90)
    assert child.returncode == 0, child.stderr
    assert len(list(tmp_path.rglob("*.so"))) == 1
