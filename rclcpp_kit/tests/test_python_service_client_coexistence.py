import os

import pytest

from _run_helper import format_output, run_helper


@pytest.mark.parametrize("order", ("service-client", "client-service"))
def test_python_service_and_native_client_coexist(
        order, monkeypatch, tmp_path):
    monkeypatch.setenv("RCLCPP_KIT_COEXISTENCE_ORDER", order)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / order))
    domain_offset = 0 if order == "service-client" else 1
    monkeypatch.setenv(
        "ROS_DOMAIN_ID", str(120 + (os.getpid() + domain_offset) % 100))
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    proc = run_helper(
        "_python_service_client_coexistence_helper.py", timeout=300)
    assert proc.returncode == 0, format_output(proc)
    assert "PYTHON_SERVICE_CLIENT_COEXISTENCE_OK order=%s" % order in proc.stdout
    assert (
        "PYTHON_SERVICE_CLIENT_COEXISTENCE_TEARDOWN_OK order=%s" % order
        in proc.stdout
    )
