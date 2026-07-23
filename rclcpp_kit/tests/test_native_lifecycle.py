import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_lifecycle import NativeLifecycleNode


class _FakeLifecycleImplementation:
    def __init__(self):
        self.is_closed = False

    def closed(self):
        return self.is_closed

    def close(self):
        self.is_closed = True

    def attached_executors(self):
        return 0


def test_close_is_idempotent_and_fences_raw_access():
    implementation = _FakeLifecycleImplementation()
    resource = NativeLifecycleNode(implementation)
    resource.close()
    resource.close()
    assert resource.closed is True
    assert resource.attached_executors == 0
    assert resource.detach_executor(object()) is False
    with pytest.raises(RuntimeError, match="closed"):
        resource.raw_node


def test_native_lifecycle_interoperates_with_stock_clients():
    proc = run_helper("_native_lifecycle_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_LIFECYCLE_OK" in proc.stdout
    assert "NATIVE_LIFECYCLE_TEARDOWN_OK" in proc.stdout


def test_native_lifecycle_transition_callbacks_and_accessors():
    proc = run_helper(
        "_native_lifecycle_transition_callback_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_LIFECYCLE_TRANSITIONS_OK" in proc.stdout
    assert "NATIVE_LIFECYCLE_TRANSITIONS_TEARDOWN_OK" in proc.stdout


def test_native_lifecycle_destroy_under_transition_dispatch():
    proc = run_helper(
        "_native_lifecycle_destroy_under_dispatch_helper.py", timeout=200)
    assert proc.returncode == 0, format_output(proc)
    assert "LIFECYCLE_DESTROY_ALL_OK" in proc.stdout
