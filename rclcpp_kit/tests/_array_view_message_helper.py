#!/usr/bin/env python3
"""Integration test for as_array() views over ROS message vector fields.

Needs a real NativeSession bringup (to resolve sensor_msgs' generated C++
headers), so -- like every other NativeSession-based test in this suite --
it runs isolated in its own subprocess (see test_array_view.py).
"""
import numpy as np

from rclcpp_kit.array_view import as_array
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


def main():
    with native(["array-view-message"]) as session:
        # A node isn't strictly needed to manipulate a message struct, but
        # NativeSession.open() (called by create_node) is what puts every
        # ament package's generated headers on cppyy's include path -- this
        # mirrors how the team uses NativeSession elsewhere in this suite.
        session.create_node("array_view_message_test")

        CppLaserScan = load_message_type("sensor_msgs", "LaserScan").cpp_type
        scan = CppLaserScan()
        scan.ranges.resize(360)
        for i in range(360):
            scan.ranges[i] = float(i) * 0.5

        ranges = as_array(scan.ranges)
        assert ranges.dtype == np.float32
        assert ranges.shape == (360,)
        assert ranges.flags.owndata is False
        assert ranges[180] == np.float32(180 * 0.5)
        assert ranges.tolist() == [float(np.float32(i * 0.5)) for i in range(360)]

        # vectorized ops at near-C speed, not element-by-element (checked
        # before the mutations below change the buffer's contents)
        expected_sum = float(np.array([np.float32(i * 0.5) for i in range(360)]).sum())
        assert abs(float(ranges.sum()) - expected_sum) < 1e-2

        # zero-copy both directions
        ranges[10] = 42.5
        assert float(scan.ranges[10]) == 42.5
        scan.ranges[20] = 7.25
        assert ranges[20] == np.float32(7.25)
        print("ARRAY_VIEW_LASER_SCAN_OK", flush=True)

        CppPointCloud2 = load_message_type("sensor_msgs", "PointCloud2").cpp_type
        cloud = CppPointCloud2()
        cloud.data.resize(32)
        for i in range(32):
            cloud.data[i] = i % 256

        data = as_array(cloud.data)
        assert data.dtype == np.uint8
        assert data.shape == (32,)
        assert data.tolist() == list(range(32))
        data[0] = 255
        # cppyy crosses a uint8_t vector element as a one-character Python
        # str, not an int (see cppyy_kit's _cpp_type docstring) -- ord() it.
        assert ord(cloud.data[0]) == 255
        print("ARRAY_VIEW_POINT_CLOUD2_OK", flush=True)


if __name__ == "__main__":
    main()
