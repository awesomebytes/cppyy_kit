import glob
import importlib.util
import os

import numpy as np
import pytest

_HAVE_PCL = bool(glob.glob(os.path.join(
    os.environ.get("CONDA_PREFIX", ""), "include", "pcl-*"
))) and importlib.util.find_spec("pcl_kit") is not None

pytestmark = pytest.mark.skipif(
    not _HAVE_PCL, reason="PCL is not installed (use the pcl environment)")

if _HAVE_PCL:
    import pcl_kit


def test_cloud_from_unaligned_float32_points():
    source = np.ndarray((3, 3), dtype=np.float32,
                        buffer=bytearray(3 * 3 * 4 + 1), offset=1)
    source[:] = [[1.0, 2.0, 3.0], [-4.0, 0.5, 6.0], [0.0, -8.0, 9.0]]
    assert source.flags.c_contiguous and not source.flags.aligned
    normalized = pcl_kit._as_f32(source)
    assert normalized.flags.c_contiguous and normalized.flags.aligned
    cloud = pcl_kit.cloud_from_numpy(source)
    actual = np.array([[point.x, point.y, point.z] for point in cloud.points],
                      dtype=np.float32)
    assert np.array_equal(actual, source)
    source[:] = 0
    retained = [[point.x, point.y, point.z] for point in cloud.points]
    assert np.array_equal(retained, actual)


def test_pointcloud_adapter_roundtrip_and_capabilities():
    source = np.array(
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [-1.0, 0.5, 9.0]],
        dtype=np.float32,
    )
    cloud = pcl_kit.cloud_from_numpy(source)
    adapter = pcl_kit.type_adapter()
    message = adapter.from_native(cloud)
    restored_cloud = adapter.to_native(message)
    restored = pcl_kit.cloud_to_numpy(restored_cloud)
    assert np.array_equal(restored, source)

    capabilities = adapter.capabilities
    assert capabilities.to_native_copy == "cpp_copy"
    assert capabilities.from_native_copy == "cpp_copy"
    assert capabilities.retains_source_owner is False
