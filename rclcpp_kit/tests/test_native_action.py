import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit import native_action
from rclcpp_kit.native_action import NativeActionClient, create_native_action_client


class _ClosedImplementation:
    def close(self):
        pass


def test_action_link_paths_include_custom_package_prefix(monkeypatch):
    monkeypatch.setattr(native_action, "get_ros2_lib_path", lambda: "/opt/ros/lib")
    monkeypatch.setattr(
        native_action,
        "get_package_prefix",
        lambda package: "/workspace/install/%s" % package,
    )
    assert native_action._action_library_paths("custom_actions") == (
        "/opt/ros/lib",
        "/workspace/install/custom_actions/lib",
    )


def test_action_link_paths_deduplicate_base_package_prefix(monkeypatch):
    monkeypatch.setattr(native_action, "get_ros2_lib_path", lambda: "/env/lib")
    monkeypatch.setattr(
        native_action, "get_package_prefix", lambda _package: "/env")
    assert native_action._action_library_paths("tf2_msgs") == ("/env/lib",)


def test_closed_action_client_rejects_new_work_without_touching_cpp():
    client = NativeActionClient(
        _ClosedImplementation(), "source", {"cached": False}, 4)
    client.close()
    assert client.closed
    assert client.server_is_ready() is False
    assert client.wait_for_server() is False
    assert client.request_cancel(1) is False
    assert client.forget(1) is False
    for operation in (
        lambda: client.raw_client,
        client.make_goal,
        lambda: client.send_goal(object()),
        lambda: client.send_cpp_value(object()),
        lambda: client.goal_response_ready(1),
        lambda: client.goal_accepted(1),
        lambda: client.raw_goal_handle(1),
        lambda: client.goal_id(1),
        lambda: client.goal_response(1),
        lambda: client.feedback_ready(1),
        lambda: client.take_feedback(1),
        lambda: client.take_feedback_message(1),
        lambda: client.result_ready(1),
        lambda: client.take_result(1),
        lambda: client.take_result_response(1),
        lambda: client.cancel_response_ready(1),
        lambda: client.take_cancel_response(1),
    ):
        with pytest.raises(RuntimeError, match="closed"):
            operation()


def test_invalid_options_are_rejected_before_compilation():
    with pytest.raises(ValueError, match="action_name"):
        create_native_action_client(None, None, object, "  ")
    with pytest.raises(ValueError, match="feedback_capacity"):
        create_native_action_client(
            None, None, object, "action", feedback_capacity=0)
    with pytest.raises(TypeError, match="ROS action class"):
        create_native_action_client(None, None, object, "action")


def test_negative_wait_timeout_is_rejected():
    client = NativeActionClient(object(), "source", {"cached": False}, 4)
    with pytest.raises(ValueError, match="non-negative"):
        client.wait_for_server(-0.1)


def test_cpp_value_submission_rejects_python_action_messages_before_cpp():
    client = NativeActionClient(object(), "source", {"cached": False}, 4)

    class PythonActionMessage:
        @staticmethod
        def get_fields_and_field_types():
            return {}

    with pytest.raises(TypeError, match="actual C\\+\\+ goal value"):
        client.send_cpp_value(PythonActionMessage())


def test_native_action_client_interoperates_with_stock_python_server():
    proc = run_helper("_native_action_helper.py", timeout=300)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_ACTION_OK" in proc.stdout
    assert "NATIVE_ACTION_TEARDOWN_OK" in proc.stdout
