import numpy as np

import pcl_kit


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
