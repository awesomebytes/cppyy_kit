import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_service import create_native_service


def test_empty_callback_is_rejected_before_type_resolution():
    with pytest.raises(ValueError, match="callback_body"):
        create_native_service(None, None, object, "service", "  ")


def test_native_service_interoperates_with_stock_client():
    proc = run_helper("_native_service_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_SERVICE_OK" in proc.stdout
    assert "NATIVE_SERVICE_TEARDOWN_OK" in proc.stdout
