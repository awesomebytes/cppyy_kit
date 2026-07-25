"""Contracts for native pre/post clock jump-callback registration."""

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_clock_jump import NativeClockJumpHandler, create_clock_jump_callback


def test_create_clock_jump_callback_requires_at_least_one_callback():
    with pytest.raises(ValueError, match="one of pre_callback or post_callback"):
        create_clock_jump_callback(
            None, on_clock_change=True, min_forward_ns=0, min_backward_ns=0)


def test_create_clock_jump_callback_rejects_non_callables():
    with pytest.raises(TypeError, match="pre_callback"):
        create_clock_jump_callback(
            None, on_clock_change=True, min_forward_ns=0, min_backward_ns=0,
            pre_callback="not callable")
    with pytest.raises(TypeError, match="post_callback"):
        create_clock_jump_callback(
            None, on_clock_change=True, min_forward_ns=0, min_backward_ns=0,
            post_callback="not callable")


class _FakeClockJumpImplementation:
    def __init__(self):
        self._closed = False

    def close(self):
        was_open = not self._closed
        self._closed = True
        return was_open

    def closed(self):
        return self._closed


def test_native_clock_jump_handler_close_is_idempotent_and_fences_closed_state():
    handler = NativeClockJumpHandler(_FakeClockJumpImplementation(), None, None)
    assert handler.closed is False
    assert handler.close() is True
    assert handler.closed is True
    assert handler.close() is False


def test_exact_clock_jump_callbacks_fire_on_simulated_time_activation_and_forward_jump():
    process = run_helper("_native_clock_jump_helper.py", timeout=180)
    assert process.returncode == 0, format_output(process)
    assert "NATIVE_CLOCK_JUMP_ACTIVATED_OK" in process.stdout
    assert "NATIVE_CLOCK_JUMP_FORWARD_OK" in process.stdout
    assert "NATIVE_CLOCK_JUMP_UNREGISTER_OK" in process.stdout
