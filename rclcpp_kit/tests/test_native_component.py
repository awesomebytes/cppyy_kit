import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_component import NativeComponentManager


class _FakeComponentManagerImplementation:
    def __init__(self):
        self.is_closed = False

    def closed(self):
        return self.is_closed

    def close(self):
        self.is_closed = True


def test_close_is_idempotent_and_fences_raw_access():
    implementation = _FakeComponentManagerImplementation()
    manager = NativeComponentManager(implementation)
    manager.close()
    manager.close()
    assert manager.closed is True
    with pytest.raises(RuntimeError, match="closed"):
        manager.raw_manager


def test_native_component_manager_loads_for_stock_clients():
    proc = run_helper("_native_component_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_COMPONENT_OK" in proc.stdout
    assert "NATIVE_COMPONENT_TEARDOWN_OK" in proc.stdout
