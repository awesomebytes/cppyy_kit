"""Independent numerical, layout, ownership and interoperability checks."""
import gc
import importlib
import os
import weakref

import numpy as np
import pytest

import buffers as b


@pytest.fixture(scope="session", autouse=True)
def native():
    b.load_native()
    # A copied missing-logic skeleton can be checked with the same test file.
    if os.environ.get("BUFFERS_KERNEL_MODULE"):
        candidate = importlib.import_module(os.environ["BUFFERS_KERNEL_MODULE"])
        b.transform_kernel = candidate.transform_kernel
        b.filter_kernel = candidate.filter_kernel


def parameters():
    # Proper rotation: +90 degrees around z, then translate in the target frame.
    return np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]), np.array([0.5, -0.2, 0.3])


def reference(points, rotation, translation, radius):
    # Explicit components avoid reproducing Eigen's expression or memory layout.
    transformed = np.column_stack([
        sum(rotation[j, k] * points[:, k] for k in range(3)) + translation[j]
        for j in range(3)])
    distances = np.sum(transformed * transformed, axis=1)
    selected = transformed[distances <= radius * radius]
    return transformed, selected, float(np.sum(selected * selected))


@pytest.mark.parametrize("n", [0, 1, 17, 10003])
def test_reference_and_sharing(n):
    points = np.random.default_rng(92).normal(size=(n, 3))
    r, t = parameters()
    result = b.process(points, r, t, 1.4)
    transformed, selected, energy = reference(points, r, t, 1.4)
    np.testing.assert_allclose(result.transformed, transformed, rtol=1e-13, atol=1e-13)
    np.testing.assert_allclose(result.points, selected, rtol=1e-13, atol=1e-13)
    assert result.energy == pytest.approx(energy, rel=2e-13, abs=1e-13)
    assert not np.shares_memory(result.transformed, points)
    assert not np.shares_memory(result.points, result.transformed)
    assert not result.points.flags.writeable
    if result.points.size:
        assert np.shares_memory(result.points, result.storage)
    assert b.pointer_kernel(result.points) == result.storage.ctypes.data
    assert b.pointer_kernel(points) == points.ctypes.data


def test_filter_boundary_order_and_all_rejected():
    points = np.array([[1., 0., 0.], [0., 0., 0.], [-1., 0., 0.], [1.01, 0., 0.]])
    result = b.process(points, np.eye(3), np.zeros(3), 1)
    np.testing.assert_array_equal(result.points, points[:3])
    assert result.energy == 2
    empty = b.process(points, np.eye(3), np.array([10., 0., 0.]), 0)
    assert empty.points.shape == (0, 3) and empty.energy == 0


def test_dense_rotation_and_arithmetic_overflow_filter():
    x, y, z = 0.37, -0.81, 1.13
    rx = np.array([[1., 0., 0.], [0., np.cos(x), -np.sin(x)], [0., np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0., np.sin(y)], [0., 1., 0.], [-np.sin(y), 0., np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0.], [np.sin(z), np.cos(z), 0.], [0., 0., 1.]])
    rotation = rx @ ry @ rz
    points = np.random.default_rng(27).normal(size=(257, 3))
    translation = np.array([0.1, 0.4, -0.6])
    result = b.process(points, rotation, translation, 1.7)
    transformed, selected, energy = reference(points, rotation, translation, 1.7)
    np.testing.assert_allclose(result.transformed, transformed, rtol=2e-13, atol=2e-13)
    np.testing.assert_allclose(result.points, selected, rtol=2e-13, atol=2e-13)
    assert result.energy == pytest.approx(energy, rel=2e-13, abs=2e-13)
    overflow = b.process(np.array([[1e308, 0., 0.]]), np.eye(3), np.array([1e308, 0., 0.]), 1e154)
    assert not np.isfinite(overflow.transformed[0, 0])
    assert overflow.points.shape == (0, 3) and overflow.energy == 0


@pytest.mark.parametrize("kind", ["slice", "negative_stride", "fortran", "float32", "endian", "unaligned"])
def test_explicit_copy_policy(kind):
    base = np.arange(36, dtype=np.float64).reshape(12, 3)
    if kind == "slice":
        points = base[::2]
    elif kind == "negative_stride":
        points = base[::-1]
    elif kind == "fortran":
        points = np.asfortranarray(base)
    elif kind == "float32":
        points = base.astype(np.float32)
    elif kind == "endian":
        points = base.astype(np.dtype(np.float64).newbyteorder("S"))
    else:
        points = np.ndarray(base.shape, dtype=np.float64, buffer=bytearray(base.nbytes + 1), offset=1)
        points[:] = base
    with pytest.raises(TypeError):
        b.BorrowedPoints(points)
    with b.BorrowedPoints(points, copy=True) as lease:
        assert lease.copied
        assert lease.array.dtype == np.float64
        assert lease.array.flags.c_contiguous and lease.array.flags.aligned
        assert not np.shares_memory(points, lease.array)
        assert lease.array.ctypes.data != points.ctypes.data
        np.testing.assert_array_equal(lease.array, points)
        result = b.process(lease, np.eye(3), np.zeros(3), 200)
        np.testing.assert_array_equal(result.points, points)


def test_readonly_is_borrowed_and_writable_output_required():
    points = np.arange(15, dtype=np.float64).reshape(5, 3)
    before = points.copy()
    points.flags.writeable = False
    with b.BorrowedPoints(points, copy=True) as lease:
        assert not lease.copied
        assert lease.array is points
        assert b.pointer_kernel(lease.array) == points.ctypes.data
        result = b.process(lease, np.eye(3), np.zeros(3), 100)
        np.testing.assert_array_equal(result.points, before)
    np.testing.assert_array_equal(points, before)
    with pytest.raises(TypeError, match="read-only"):
        b.transform_kernel(points, np.eye(3), np.zeros(3), points)


@pytest.mark.parametrize("value", [np.ones((3, 2)), np.ones(3), np.ones((2, 3), dtype=np.int64),
                                  np.ones((2, 3), dtype=np.float16), np.full((2, 3), np.nan),
                                  np.full((2, 3), np.inf), [[1., 2., 3.]]])
def test_invalid_input(value):
    with pytest.raises((TypeError, ValueError)):
        b.BorrowedPoints(value, copy=True)


def test_owner_lifetime_close_and_layout_change():
    owner = np.arange(30, dtype=np.float64).reshape(10, 3).copy()
    ref = weakref.ref(owner)
    lease = b.BorrowedPoints(owner)
    del owner
    gc.collect()
    assert ref() is not None
    assert b.energy_kernel(lease.array) == np.sum(np.arange(30, dtype=np.float64) ** 2)
    # Default NumPy refcheck rejects reallocation while references exist.
    with pytest.raises(ValueError):
        lease.array.resize((100, 3))
    # Change only metadata. Validate catches it before constructing a native Map.
    with pytest.warns(DeprecationWarning, match="shape"):
        lease.array.shape = (5, 6)
    with pytest.raises(RuntimeError, match="layout changed"):
        b.process(lease, np.eye(3), np.zeros(3), 100)
    lease.close()
    gc.collect()
    assert ref() is None
    with pytest.raises(RuntimeError, match="closed"):
        _ = lease.array
    lease.close()


def test_native_output_lives_after_result_is_deleted():
    result = b.process(np.ones((4, 3)), np.eye(3), np.zeros(3), 2)
    storage_ref = weakref.ref(result.storage)
    points = result.points
    del result
    gc.collect()
    assert storage_ref() is not None
    assert b.energy_kernel(points) == 12
    del points
    gc.collect()
    assert storage_ref() is None


def test_cpu_dlpack_pointer_and_owner_lifetime():
    producer = np.arange(30, dtype=np.float64).reshape(10, 3).copy()
    reference_values = producer.copy()
    producer_ref = weakref.ref(producer)
    pointer = producer.ctypes.data
    lease = b.dlpack_points(producer)
    assert lease.array.ctypes.data == pointer
    assert np.shares_memory(lease.array, producer)
    assert b.pointer_kernel(lease.array) == pointer
    del producer
    gc.collect()
    assert producer_ref() is not None
    result = b.process(lease, np.eye(3), np.zeros(3), 100)
    np.testing.assert_array_equal(result.points, reference_values)
    lease.close()
    gc.collect()
    assert producer_ref() is None


def test_dlpack_layout_rejection_and_device_policy():
    producer = np.arange(30, dtype=np.float64).reshape(10, 3)[::2]
    imported = np.from_dlpack(producer, copy=False)
    assert imported.ctypes.data == producer.ctypes.data
    assert not imported.flags.c_contiguous
    with pytest.raises(TypeError):
        b.dlpack_points(producer)
    class FakeGPU:
        def __dlpack_device__(self):
            return (2, 0)
    with pytest.raises(ValueError, match="CPU"):
        b.dlpack_points(FakeGPU())


@pytest.mark.parametrize("r,t,radius", [(np.eye(3) * 2, np.zeros(3), 1),
                                       (np.diag([1., 1., -1.]), np.zeros(3), 1),
                                       (np.eye(3), np.zeros(3), -1),
                                       (np.eye(3), np.zeros(3), np.inf),
                                       (np.eye(3), np.zeros(3), 1e300),
                                       (np.eye(3), np.full(3, np.nan), 1)])
def test_bad_rigid_parameters(r, t, radius):
    with pytest.raises(ValueError):
        b.process(np.zeros((1, 3)), r, t, radius)


def test_native_size_validation():
    with pytest.raises(Exception, match="buffer size"):
        b.energy_kernel(np.array([1., 2.]))
    with pytest.raises(Exception, match="buffer sizes"):
        b.transform_kernel(np.zeros((2, 3)), np.eye(3), np.zeros(3), np.zeros((1, 3)))
    with pytest.raises(Exception, match="buffer sizes"):
        b.filter_kernel(np.zeros((2, 3)), 1., np.zeros((1, 3)))
