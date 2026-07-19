"""Contracts for exact session-owned node clock access."""

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native import NativeSession
from rclcpp_kit.native_clock import NativeNodeClock


def test_unowned_node_fails_before_clock_creation():
    session = NativeSession()
    with pytest.raises(ValueError, match="not owned"):
        session.create_native_node_clock(object())


class _FakeClockImplementation:
    def __init__(self):
        self._closed = False

    def close(self):
        was_open = not self._closed
        self._closed = True
        return was_open

    def closed(self):
        return self._closed

    def raw_clock(self):
        raise AssertionError("touched C++ clock after close")

    def address(self):
        raise AssertionError("touched C++ clock after close")

    def clock_type(self):
        raise AssertionError("touched C++ clock after close")

    def ros_time_is_active(self):
        raise AssertionError("touched C++ clock after close")

    def now(self):
        raise AssertionError("touched C++ clock after close")

    def now_nanoseconds(self):
        raise AssertionError("touched C++ clock after close")


def test_closed_native_node_clock_fences_access_without_touching_cpp():
    clock = NativeNodeClock(_FakeClockImplementation())
    assert clock.close() is True
    assert clock.close() is False
    assert clock.closed is True
    for accessor in (
        lambda: clock.raw_clock,
        lambda: clock.address,
        lambda: clock.clock_type,
        lambda: clock.ros_time_is_active,
        clock.now,
        clock.now_nanoseconds,
    ):
        with pytest.raises(RuntimeError, match="closed"):
            accessor()


def test_exact_node_clock_simulated_time_and_lifecycle():
    process = run_helper("_native_clock_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "NATIVE_NODE_CLOCK_EXACT_TIME_OK" in process.stdout
    assert "NATIVE_NODE_CLOCK_USE_SIM_TIME_OK" in process.stdout
    assert "NATIVE_NODE_CLOCK_LIFECYCLE_OK" in process.stdout
    assert "NATIVE_NODE_CLOCK_NO_CONVERSION_OK" in process.stdout
