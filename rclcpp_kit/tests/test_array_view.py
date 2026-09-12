"""Tests for rclcpp_kit.array_view (zero-copy NumPy views over std::vector<T>).

Correctness against generic ``std::vector<T>`` (dtype coverage, zero-copy
read/write, empty vectors, lifetime safety, resize invalidation, and the
fail-closed type guard) needs only cppyy -- no ROS/rclcpp bringup -- so those
run in-process for speed. Proving the same utility against real ROS message
fields (``LaserScan.ranges``, a ``PointCloud2``-style byte buffer) needs a
real ``NativeSession`` bringup, which -- like every other NativeSession test
in this suite -- runs isolated in its own subprocess via ``run_helper`` (see
``_direct_generic_message_helper.py`` for the established pattern).
"""
import gc

import cppyy
import numpy as np
import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.array_view import VectorArrayView, as_array

cppyy.cppdef(
    """
    #include <cstdint>
    #include <string>
    #include <vector>

    namespace array_view_test {

    struct SubMessage { int x; };

    template <typename T>
    std::vector<T> make_vector(int count) {
        std::vector<T> v;
        v.reserve(static_cast<size_t>(count));
        for (int i = 0; i < count; ++i) {
            v.push_back(static_cast<T>(i + 1));
        }
        return v;
    }

    std::vector<bool> make_bool_vector() { return {true, false, true}; }
    std::vector<std::string> make_string_vector() { return {"a", "b"}; }
    std::vector<SubMessage> make_submessage_vector() {
        return {SubMessage{1}, SubMessage{2}};
    }

    }  // namespace array_view_test
    """
)

_NS = cppyy.gbl.array_view_test

# label -> (C++ element type, expected numpy dtype)
_NUMERIC_TYPES = {
    "float32": ("float", np.float32),
    "float64": ("double", np.float64),
    "int8": ("int8_t", np.int8),
    "uint8": ("uint8_t", np.uint8),
    "int16": ("int16_t", np.int16),
    "uint16": ("uint16_t", np.uint16),
    "int32": ("int32_t", np.int32),
    "uint32": ("uint32_t", np.uint32),
    "int64": ("int64_t", np.int64),
    "uint64": ("uint64_t", np.uint64),
}


def _make_vector(cpp_elem_type, count=5):
    return _NS.make_vector[cpp_elem_type](count)


# --- dtype coverage ----------------------------------------------------
@pytest.mark.parametrize(
    ("label", "cpp_elem_type", "expected_dtype"),
    [(label, cpp_t, dtype) for label, (cpp_t, dtype) in _NUMERIC_TYPES.items()],
)
def test_all_supported_ros_numeric_types(label, cpp_elem_type, expected_dtype):
    vector = _make_vector(cpp_elem_type, count=5)
    arr = as_array(vector)
    assert isinstance(arr, np.ndarray)
    assert isinstance(arr, VectorArrayView)
    assert arr.dtype == np.dtype(expected_dtype)
    assert arr.tolist() == [1, 2, 3, 4, 5]


# --- zero copy: both directions -----------------------------------------
def test_numpy_write_is_visible_through_the_vector():
    vector = _make_vector("float", count=5)
    arr = as_array(vector)
    arr[2] = 999.5
    assert float(vector[2]) == 999.5


def test_vector_write_is_visible_through_numpy():
    vector = _make_vector("float", count=5)
    arr = as_array(vector)
    vector[3] = 123.5
    assert arr[3] == pytest.approx(123.5)


def test_array_does_not_own_its_data():
    vector = _make_vector("double", count=4)
    arr = as_array(vector)
    assert arr.flags.owndata is False
    assert arr.flags.writeable is True


# --- empty vector --------------------------------------------------------
def test_empty_vector_returns_empty_typed_array():
    vector = _make_vector("float", count=0)
    arr = as_array(vector)
    assert arr.shape == (0,)
    assert arr.dtype == np.float32
    assert arr.tolist() == []


# --- lifetime safety: vector proxy kept alive ----------------------------
def test_array_survives_gc_of_every_other_vector_reference():
    """Dropping every Python name bound to the vector must not free its
    buffer while an as_array() view is still reachable (see array_view's
    module docstring: VectorArrayView holds the keep-alive reference)."""

    def make():
        vector = _make_vector("double", count=6)
        return as_array(vector)  # `vector` goes out of scope on return

    arr = make()
    gc.collect()
    # Pressure the allocator so a freed buffer would likely be overwritten.
    junk = [bytearray(b"\xff" * 64) for _ in range(4000)]
    gc.collect()
    del junk
    assert arr.tolist() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_slice_of_the_view_keeps_the_reference_too():
    def make():
        vector = _make_vector("int32_t", count=6)
        return as_array(vector)[1:4]

    sub = make()
    gc.collect()
    junk = [bytearray(b"\xff" * 64) for _ in range(4000)]
    gc.collect()
    del junk
    assert sub.tolist() == [2, 3, 4]


# --- resize invalidation (documented, not preventable) --------------------
def test_resize_past_capacity_invalidates_the_existing_view():
    """A view created before a reallocating mutation keeps pointing at the
    old (now-freed) buffer address -- it does not track the vector's new
    buffer. This asserts the documented contract (re-call as_array() after
    a resize) by comparing buffer addresses rather than reading the stale,
    UB memory contents."""
    vector = _NS.make_vector["float"](4)
    assert int(vector.capacity()) == 4
    stale = as_array(vector)
    stale_addr = stale.__array_interface__["data"][0]

    # Push past capacity: guaranteed reallocation.
    vector.push_back(99.0)
    vector.push_back(98.0)
    assert int(vector.capacity()) > 4

    fresh = as_array(vector)
    fresh_addr = fresh.__array_interface__["data"][0]
    assert fresh_addr != stale_addr
    assert fresh.tolist() == [1.0, 2.0, 3.0, 4.0, 99.0, 98.0]


# --- fail-closed on unsupported element types -----------------------------
def test_bool_vector_is_rejected():
    vector = _NS.make_bool_vector()
    with pytest.raises(TypeError, match="fixed-width numeric"):
        as_array(vector)


def test_string_vector_is_rejected():
    vector = _NS.make_string_vector()
    with pytest.raises(TypeError, match="fixed-width numeric"):
        as_array(vector)


def test_submessage_vector_is_rejected():
    vector = _NS.make_submessage_vector()
    with pytest.raises(TypeError, match="fixed-width numeric"):
        as_array(vector)


# --- real ROS message types (needs rclcpp bringup -> subprocess) ---------
def test_real_ros_message_vector_fields():
    process = run_helper("_array_view_message_helper.py")
    assert process.returncode == 0, format_output(process)
    assert "ARRAY_VIEW_LASER_SCAN_OK" in process.stdout
    assert "ARRAY_VIEW_POINT_CLOUD2_OK" in process.stdout


# --- benchmark -------------------------------------------------------------
def test_benchmark_laser_scan_ranges():
    process = run_helper("_array_view_benchmark_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "ARRAY_VIEW_BENCHMARK_OK" in process.stdout
