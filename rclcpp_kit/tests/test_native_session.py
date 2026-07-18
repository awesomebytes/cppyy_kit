import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native import NativeCapabilities, NativeSession, native


def test_capabilities_are_structured_and_conservative():
    report = NativeCapabilities().to_dict()
    assert report["managed_context"] is True
    assert report["intra_process"] is True
    assert report["loaned_messages"] == "publisher_runtime_query"
    assert report["raw_rclcpp"] is True


def test_native_factory_is_lazy():
    session = native(["program"])
    assert isinstance(session, NativeSession)
    assert session.closed is False
    assert session.nodes == ()
    assert session.executors == ()


@pytest.mark.parametrize("kind", ["bad", "events"])
def test_unknown_executor_kind_fails_before_creation(kind):
    session = NativeSession()
    session._context = type("Context", (), {"is_valid": lambda self: True})()
    session._rclcpp = object()
    with pytest.raises(ValueError, match="executor kind"):
        session.create_executor(kind)


def test_managed_context_pubsub_and_teardown():
    proc = run_helper("_native_session_helper.py")
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_SESSION_OK" in proc.stdout
    assert "NATIVE_TEARDOWN_OK" in proc.stdout
