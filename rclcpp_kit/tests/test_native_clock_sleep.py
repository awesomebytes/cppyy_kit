"""Contracts for the ROS-time-aware, context-interruptible clock-sleep primitive."""

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native import NativeSession
from rclcpp_kit.native_clock_sleep import NativeClockSleeper


def test_unowned_node_fails_before_sleeper_creation():
    session = NativeSession()
    with pytest.raises(ValueError, match="not owned"):
        session.create_native_clock_sleeper(object())


class _FakeClockSleeperImplementation:
    def __init__(self):
        self._closed = False

    def close(self):
        was_open = not self._closed
        self._closed = True
        return was_open

    def closed(self):
        return self._closed

    def sleep_for(self, *_):
        raise AssertionError("touched C++ clock sleeper after close")

    def sleep_until(self, *_):
        raise AssertionError("touched C++ clock sleeper after close")

    def clock_address(self):
        raise AssertionError("touched C++ clock sleeper after close")


def test_closed_native_clock_sleeper_fences_access_without_touching_cpp():
    sleeper = NativeClockSleeper(_FakeClockSleeperImplementation())
    assert sleeper.close() is True
    assert sleeper.close() is False
    assert sleeper.closed is True
    with pytest.raises(RuntimeError, match="closed"):
        sleeper.sleep_for(1)
    with pytest.raises(RuntimeError, match="closed"):
        sleeper.sleep_until(1)
    with pytest.raises(RuntimeError, match="closed"):
        sleeper.clock_address


def test_native_clock_sleeper_wall_sim_and_interrupt():
    process = run_helper("_native_clock_sleep_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "NATIVE_CLOCK_SLEEP_WALL_OK" in process.stdout
    assert "NATIVE_CLOCK_SLEEP_IDENTITY_OK" in process.stdout
    assert "NATIVE_CLOCK_SLEEP_SIM_OK" in process.stdout
    assert "NATIVE_CLOCK_SLEEP_INTERRUPT_OK" in process.stdout
