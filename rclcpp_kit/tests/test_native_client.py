import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit import native_client
from rclcpp_kit.native_client import NativeClient


class _ClosedImplementation:
    def close(self):
        pass


def test_closed_client_rejects_new_work_without_touching_cpp():
    client = NativeClient(_ClosedImplementation(), "source", {"cached": False})
    client.close()
    assert client.closed
    assert client.service_is_ready() is False
    assert client.wait_for_service() is False
    assert client.cancel(1) is False
    with pytest.raises(RuntimeError, match="closed"):
        client.make_request()
    with pytest.raises(RuntimeError, match="closed"):
        client.send(object())
    with pytest.raises(RuntimeError, match="closed"):
        client.send_cpp_value(object())
    with pytest.raises(RuntimeError, match="closed"):
        client.ready(1)
    with pytest.raises(RuntimeError, match="closed"):
        client.take(1)


def test_negative_wait_timeout_is_rejected():
    client = NativeClient(object(), "source", {"cached": False})
    with pytest.raises(ValueError, match="non-negative"):
        client.wait_for_service(-0.1)


def test_send_rejects_python_requests_before_native_or_conversion(monkeypatch):
    calls = []
    assert not hasattr(native_client, "convert_python_msg_to_cpp")

    class Implementation:
        def make_request(self):
            calls.append("make_request")
            return object()

        def send(self, request):
            calls.append(("send", request))
            return 7

    class PythonRequest:
        @staticmethod
        def get_fields_and_field_types():
            return {}

    def forbidden_conversion(*_args, **_kwargs):
        raise AssertionError("Python request conversion ran")

    monkeypatch.setattr(
        native_client,
        "convert_python_msg_to_cpp",
        forbidden_conversion,
        raising=False,
    )
    client = NativeClient(Implementation(), "source", {"cached": False})

    with pytest.raises(TypeError, match="shared C\\+\\+ request.*make_request"):
        client.send(PythonRequest())
    assert calls == []

    shared_request = object()
    assert client.send(shared_request) == 7
    assert calls == [("send", shared_request)]


def test_native_client_interoperates_with_stock_python_server():
    proc = run_helper("_native_client_helper.py", timeout=240)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_CLIENT_OK" in proc.stdout
    assert "NATIVE_CLIENT_TEARDOWN_OK" in proc.stdout
