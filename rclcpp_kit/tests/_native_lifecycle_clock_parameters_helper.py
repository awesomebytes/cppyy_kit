#!/usr/bin/env python3
"""Test PLAN-lifecycle.md S5: a lifecycle node's own clock and
parameter surfaces work directly.

create_lifecycle_node_clock is a thin lifecycle-node-typed twin of
native_clock.create_native_node_clock (rclcpp_lifecycle::LifecycleNode does
not inherit rclcpp::Node, so the base NativeNodeClock C++ implementation
cannot bind it directly). ``LifecycleNode::get_clock()`` behaves like
``Node::get_clock()``.

The parameter methods need no changes because the lifecycle node exposes the
same ``declare_parameter``, ``get_parameter``, ``set_parameters``,
``has_parameter``, and ``undeclare_parameter`` methods as ``rclcpp::Node``
(see ``lifecycle_node.hpp``). This test checks that API.
"""
import time

from rclcpp_kit import native_parameters
from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import create_lifecycle_node_clock


def main():
    with native(["native-lifecycle-clock-parameters-test"]) as ros:
        lifecycle = ros.create_native_lifecycle_node(
            "native_lifecycle_clock_parameters")
        raw_node = lifecycle.raw_node

        clock = create_lifecycle_node_clock(raw_node)
        assert clock.closed is False
        first_ns = clock.now_nanoseconds()
        time.sleep(0.05)
        second_ns = clock.now_nanoseconds()
        assert second_ns > first_ns
        assert isinstance(clock.address, int) and clock.address != 0
        assert clock.clock_type in (1, 2, 3)
        assert clock.ros_time_is_active in (True, False)
        assert clock.now().nanoseconds() >= second_ns

        declared = native_parameters.declare_parameter(
            raw_node, native_parameters.parameter_integer("data_plane", 42))
        assert declared.name == "data_plane"
        assert declared.value_snapshot() == 42
        assert native_parameters.has_parameter(raw_node, "data_plane") is True

        fetched = native_parameters.get_parameter(raw_node, "data_plane")
        assert fetched.value_snapshot() == 42

        results = native_parameters.set_parameters(
            raw_node,
            [native_parameters.parameter_integer("data_plane", 99)],
        )
        assert len(results) == 1
        assert bool(results[0].successful) is True

        refetched = native_parameters.get_parameter(raw_node, "data_plane")
        assert refetched.value_snapshot() == 99

        assert clock.close() is True
        assert clock.closed is True
        assert clock.close() is False
        try:
            clock.now()
        except RuntimeError as exc:
            assert "closed" in str(exc)
        else:
            raise AssertionError("closed lifecycle node clock remained usable")

        lifecycle.close()
        assert lifecycle.closed is True
        print("NATIVE_LIFECYCLE_CLOCK_PARAMETERS_OK")


if __name__ == "__main__":
    main()
