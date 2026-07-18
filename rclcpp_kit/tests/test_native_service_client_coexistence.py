import os

import pytest

from _run_helper import format_output, run_helper


@pytest.mark.parametrize("order", ("service-client", "client-service"))
def test_native_service_and_client_coexist_in_one_interpreter(
        order, monkeypatch, tmp_path):
    monkeypatch.setenv("RCLCPP_KIT_COEXISTENCE_ORDER", order)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / order))
    domain_offset = 0 if order == "service-client" else 1
    monkeypatch.setenv(
        "ROS_DOMAIN_ID", str(100 + (os.getpid() + domain_offset) % 100))
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    proc = run_helper(
        "_native_service_client_coexistence_helper.py", timeout=300)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_SERVICE_CLIENT_COEXISTENCE_OK order=%s" % order in proc.stdout
    assert "NATIVE_SERVICE_CLIENT_COEXISTENCE_TEARDOWN_OK order=%s" % order in proc.stdout
