import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native import NativeCapabilities, NativeSession, native


def test_capabilities_are_structured_and_conservative():
    report = NativeCapabilities().to_dict()
    assert report["managed_context"] is True
    assert report["managed_executor_thread"] is True
    assert report["callback_group_entity_options"] is True
    assert report["managed_native_services"] is True
    assert report["managed_python_services"] is True
    assert report["managed_borrowed_set_bool_services"] is True
    assert report["managed_native_clients"] is True
    assert report["native_service_client_coexistence"] == (
        "runtime_compiler_or_warm_cache")
    assert report["managed_native_action_clients"] is True
    assert report["managed_native_action_servers"] is True
    assert report["managed_node_clock"] is True
    assert report["managed_guard_conditions"] is True
    assert report["managed_wait_sets"] is True
    assert report["managed_clock_sleep"] is True
    assert report["managed_lifecycle_nodes"] is True
    assert report["managed_component_containers"] is True
    assert report["intra_process"] is True
    assert report["direct_cpp_message_entities"] is True
    assert report["raw_node_options"] is True
    assert report["raw_qos_profiles"] is True
    assert report["actual_qos_introspection"] == (
        "publisher_and_subscription_runtime_query")
    assert report["managed_entity_options_proof"] == "tested_axes_only"
    assert report["arbitrary_entity_option_combinations"] == "unknown"
    assert report["loaned_messages"] == "publisher_runtime_query"
    assert report["raw_rclcpp"] is True


def test_native_factory_is_lazy():
    session = native(["program"])
    assert isinstance(session, NativeSession)
    assert session.closed is False
    assert session.nodes == ()
    assert session.executors == ()
    assert session.executor_threads == ()
    assert session.resources == ()


@pytest.mark.parametrize("kind", ["bad", "events"])
def test_unknown_executor_kind_fails_before_creation(kind):
    session = NativeSession()
    session._context = type("Context", (), {"is_valid": lambda self: True})()
    session._rclcpp = object()
    with pytest.raises(ValueError, match="executor kind"):
        session.create_executor(kind)


def test_release_node_rejects_unowned_node():
    session = NativeSession()
    session._context = type("Context", (), {"is_valid": lambda self: True})()
    session._rclcpp = object()
    with pytest.raises(ValueError, match="not owned"):
        session.release_node(object())


def test_managed_context_pubsub_and_teardown():
    proc = run_helper("_native_session_helper.py")
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_SESSION_OK" in proc.stdout
    assert "NATIVE_TEARDOWN_OK" in proc.stdout
