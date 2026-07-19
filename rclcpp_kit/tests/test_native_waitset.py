"""Contracts for native guard-condition and wait-set wake behavior."""

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_waitset import NativeGuardCondition, NativeWaitSet


class _FakeGuardImplementation:
    def __init__(self):
        self._closed = False

    def close(self):
        was_open = not self._closed
        self._closed = True
        return was_open

    def closed(self):
        return self._closed

    def trigger(self):
        raise AssertionError("touched C++ guard after close")

    def raw_guard_condition(self):
        raise AssertionError("touched C++ guard after close")

    def address(self):
        raise AssertionError("touched C++ guard after close")


def test_closed_native_guard_condition_fences_access_without_touching_cpp():
    guard = NativeGuardCondition(_FakeGuardImplementation())
    assert guard.close() is True
    assert guard.close() is False
    assert guard.closed is True
    with pytest.raises(RuntimeError, match="closed"):
        guard.trigger()
    for accessor in (lambda: guard.raw_guard_condition, lambda: guard.address):
        with pytest.raises(RuntimeError, match="closed"):
            accessor()


class _FakeWaitSetImplementation:
    def __init__(self):
        self._closed = False

    def close(self):
        was_open = not self._closed
        self._closed = True
        return was_open

    def closed(self):
        return self._closed

    def add_guard_condition(self, *_):
        raise AssertionError("touched C++ wait set after close")

    def wait_kind(self, *_):
        raise AssertionError("touched C++ wait set after close")

    def raw_wait_set(self):
        raise AssertionError("touched C++ wait set after close")

    def address(self):
        raise AssertionError("touched C++ wait set after close")


def test_closed_native_wait_set_fences_access_without_touching_cpp():
    wait_set = NativeWaitSet(_FakeWaitSetImplementation())
    assert wait_set.close() is True
    assert wait_set.close() is False
    assert wait_set.closed is True
    with pytest.raises(RuntimeError, match="closed"):
        wait_set.add_guard_condition(object())
    with pytest.raises(RuntimeError, match="closed"):
        wait_set.wait()
    for accessor in (lambda: wait_set.raw_wait_set, lambda: wait_set.address):
        with pytest.raises(RuntimeError, match="closed"):
            accessor()


def test_native_guard_condition_wakes_wait_set():
    process = run_helper("_native_waitset_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "NATIVE_WAITSET_IDENTITY_OK" in process.stdout
    assert "NATIVE_WAITSET_LATCH_OK" in process.stdout
    assert "NATIVE_WAITSET_CROSS_THREAD_WAKE_OK" in process.stdout
    assert "NATIVE_WAITSET_EXCLUSIVE_OK" in process.stdout
    assert "NATIVE_WAITSET_TIMEOUT_OK" in process.stdout
    assert "NATIVE_WAITSET_LIFECYCLE_OK" in process.stdout
