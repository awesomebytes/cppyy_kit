import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.python_service import create_python_service


def test_noncallable_callback_is_rejected_before_type_resolution():
    with pytest.raises(TypeError, match="callback"):
        create_python_service(None, None, object, "service", None)


def test_coroutine_callback_is_rejected_before_type_resolution():
    async def callback(_request, _response):
        return None

    with pytest.raises(TypeError, match="synchronous"):
        create_python_service(None, None, object, "service", callback)


def test_python_service_interoperates_with_stock_client():
    proc = run_helper("_python_service_helper.py", timeout=300)
    assert proc.returncode == 0, format_output(proc)
    assert "PYTHON_SERVICE_OK" in proc.stdout
    assert "PYTHON_SERVICE_TEARDOWN_OK" in proc.stdout
