#!/usr/bin/env python3
"""Live exact-C++ action-server proof on Jazzy with Cyclone DDS."""

import gc
import importlib
import os
import time

from action_msgs.msg import GoalStatus
import cppyy
from rclcpp_kit.native import native
from rclcpp_kit.native_action import resolve_cpp_action_type
from tf2_msgs.action import LookupTransform


def as_int8(value):
    return ord(value) if isinstance(value, str) else int(value)


def main():
    assert os.environ.get("ROS_DISTRO") == "jazzy"
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp"

    def forbidden_boundary(*_args, **_kwargs):
        raise AssertionError("a Python conversion or serialization boundary ran")

    native_action = importlib.import_module("rclcpp_kit.native_action")
    kit = importlib.import_module("rclcpp_kit")
    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    serialization = importlib.import_module("rclcpp_kit.serialization")
    rclpy_serialization = importlib.import_module("rclpy.serialization")
    kit.convert_python_msg_to_cpp = forbidden_boundary
    native_action.convert_python_msg_to_cpp = forbidden_boundary
    bringup.convert_python_msg_to_cpp = forbidden_boundary
    serialization.serialize_message = forbidden_boundary
    serialization.deserialize_message = forbidden_boundary
    serialization.serialized_message_from_bytes = forbidden_boundary
    serialization.serialized_message_to_bytes = forbidden_boundary
    rclpy_serialization.serialize_message = forbidden_boundary
    rclpy_serialization.deserialize_message = forbidden_boundary

    retained = []
    cancel_tokens = []
    cancel_error_tokens = set()
    with native(["native-action-server-test"]) as ros:
        cpp_types = resolve_cpp_action_type(LookupTransform)
        node = ros.create_node("native_action_server_test")
        group = ros.create_callback_group(node, "mutually_exclusive")
        reentrant_group = ros.create_callback_group(node, "reentrant")
        executor = ros.create_executor("single_threaded")
        executor.add_node(node)

        try:
            ros.create_native_action_server(
                node,
                LookupTransform,
                "native_reentrant_server",
                callback_group=reentrant_group,
            )
        except TypeError as error:
            assert "mutually exclusive" in str(error)
        else:
            raise AssertionError("P0 server accepted a reentrant callback group")

        def goal_callback(goal):
            assert type(goal) is cpp_types.goal
            assert not hasattr(goal, "get_fields_and_field_types")
            retained.append(goal)
            target = str(goal.target_frame)
            if target == "error":
                raise RuntimeError("contained goal decision error")
            if target == "invalid":
                return "not a bool"
            return target != "reject"

        def cancel_callback(token):
            cancel_tokens.append(token)
            if token in cancel_error_tokens:
                raise RuntimeError("contained cancel decision error")
            return True

        server = ros.create_native_action_server(
            node,
            LookupTransform,
            "native_server_lookup",
            goal_callback=goal_callback,
            cancel_callback=cancel_callback,
            callback_group=group,
            result_timeout=5.0,
        )
        del goal_callback, cancel_callback
        gc.collect()
        client = ros.create_native_action_client(
            node,
            LookupTransform,
            "native_server_lookup",
            feedback_capacity=8,
        )

        duration = cppyy.gbl.std.chrono.milliseconds(10)

        def spin_until(predicate, label, timeout=12.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(duration)
            assert predicate(), "timed out waiting for %s" % label

        assert client.wait_for_server(5.0)
        assert server.raw_server

        successful = cpp_types.goal()
        successful.target_frame = "success"
        successful.source_frame = "base"
        successful_client_token = client.send_cpp_value(successful)
        spin_until(
            lambda: client.goal_response_ready(successful_client_token),
            "successful goal response",
        )
        assert client.goal_accepted(successful_client_token)
        assert server.accepted_ready_count() == 1
        successful_server_goal = server.take_accepted()
        assert type(successful_server_goal.goal) is cpp_types.goal
        assert type(successful_server_goal.goal_id) is cpp_types.goal_id
        assert str(successful_server_goal.goal.target_frame) == "success"
        assert any(int(item) for item in successful_server_goal.goal_id.uuid)
        retained.extend((successful_server_goal.goal, successful_server_goal.goal_id))

        premature = cpp_types.result()
        try:
            server.succeed(successful_server_goal.token, premature)
        except RuntimeError:
            pass
        else:
            raise AssertionError("succeed before execute crossed as a valid operation")
        assert server.status(successful_server_goal.token) == GoalStatus.STATUS_ACCEPTED
        server.execute(successful_server_goal.token)
        assert server.is_executing(successful_server_goal.token)
        time.sleep(0.1)
        feedback = cpp_types.feedback()
        server.publish_feedback(successful_server_goal.token, feedback)
        shared_feedback = server.make_feedback_shared()
        assert type(shared_feedback) is cpp_types.feedback
        assert bool(shared_feedback.__smartptr__())
        server.publish_feedback_shared(
            successful_server_goal.token, shared_feedback)
        try:
            server.publish_feedback_shared(
                successful_server_goal.token, cpp_types.feedback())
        except TypeError as error:
            assert "shared factory" in str(error)
        else:
            raise AssertionError("ordinary feedback used the shared handoff")
        shared_result = server.make_result_shared()
        assert type(shared_result) is cpp_types.result
        assert bool(shared_result.__smartptr__())
        shared_result.transform.child_frame_id = "success-result"
        try:
            server.succeed_shared(
                successful_server_goal.token, cpp_types.result())
        except TypeError as error:
            assert "shared factory" in str(error)
        else:
            raise AssertionError("ordinary result used the shared handoff")
        server.succeed_shared(successful_server_goal.token, shared_result)
        spin_until(
            lambda: client.result_ready(successful_client_token),
            "successful result",
        )
        spin_until(
            lambda: client.stats().feedback_received == 2,
            "successful feedback",
        )
        feedback_messages = [
            client.take_feedback_message(successful_client_token)
            for _ in range(2)
        ]
        assert all(type(item) is cpp_types.feedback_message for item in feedback_messages)
        assert all(type(item.feedback) is cpp_types.feedback for item in feedback_messages)
        successful_response = client.take_result_response(successful_client_token)
        assert type(successful_response) is cpp_types.result_response
        assert as_int8(successful_response.status) == GoalStatus.STATUS_SUCCEEDED
        assert str(successful_response.result.transform.child_frame_id) == "success-result"
        retained.extend((feedback, shared_feedback, shared_result, *feedback_messages))
        retained.append(successful_response)

        for target, expected_error in (
            ("reject", None),
            ("error", RuntimeError),
            ("invalid", TypeError),
        ):
            rejected_goal = cpp_types.goal()
            rejected_goal.target_frame = target
            rejected_goal.source_frame = "base"
            rejected_token = client.send_cpp_value(rejected_goal)
            spin_until(
                lambda token=rejected_token: client.goal_response_ready(token),
                "%s goal response" % target,
            )
            assert not client.goal_accepted(rejected_token)
            assert server.accepted_ready_count() == 0
            assert client.forget(rejected_token)
            if expected_error is not None:
                assert server.callback_error_ready()
                error = server.take_callback_error()
                assert type(error) is expected_error
        assert not server.callback_error_ready()

        cancel_goal = cpp_types.goal()
        cancel_goal.target_frame = "cancel"
        cancel_goal.source_frame = "base"
        cancel_client_token = client.send_cpp_value(cancel_goal)
        spin_until(
            lambda: client.goal_response_ready(cancel_client_token),
            "cancel goal response",
        )
        cancel_server_goal = server.take_accepted()
        assert server.is_active(cancel_server_goal.token)
        gc.collect()
        assert server.is_active(cancel_server_goal.token)
        assert client.request_cancel(cancel_client_token)
        spin_until(
            lambda: client.cancel_response_ready(cancel_client_token),
            "cancel response",
        )
        cancel_response = client.take_cancel_response(cancel_client_token)
        assert type(cancel_response) is cpp_types.cancel_response
        assert len(cancel_response.goals_canceling) == 1
        assert cancel_tokens == [cancel_server_goal.token]
        assert server.is_canceling(cancel_server_goal.token)
        cancel_result = cpp_types.result()
        cancel_result.transform.child_frame_id = "cancel-result"
        server.canceled(cancel_server_goal.token, cancel_result)
        spin_until(
            lambda: client.result_ready(cancel_client_token),
            "canceled result",
        )
        canceled_response = client.take_result_response(cancel_client_token)
        assert as_int8(canceled_response.status) == GoalStatus.STATUS_CANCELED
        assert str(canceled_response.result.transform.child_frame_id) == "cancel-result"

        cancel_error_goal = cpp_types.goal()
        cancel_error_goal.target_frame = "cancel-error"
        cancel_error_goal.source_frame = "base"
        cancel_error_client_token = client.send_cpp_value(cancel_error_goal)
        spin_until(
            lambda: client.goal_response_ready(cancel_error_client_token),
            "cancel-error goal response",
        )
        cancel_error_server_goal = server.take_accepted()
        cancel_error_tokens.add(cancel_error_server_goal.token)
        assert client.request_cancel(cancel_error_client_token)
        spin_until(
            lambda: client.cancel_response_ready(cancel_error_client_token),
            "cancel-error response",
        )
        rejected_cancel = client.take_cancel_response(cancel_error_client_token)
        assert len(rejected_cancel.goals_canceling) == 0
        assert server.callback_error_ready()
        assert type(server.take_callback_error()) is RuntimeError
        assert server.status(cancel_error_server_goal.token) == \
            GoalStatus.STATUS_ACCEPTED
        server.execute(cancel_error_server_goal.token)
        recovered_result = cpp_types.result()
        recovered_result.transform.child_frame_id = "cancel-error-abort"
        server.abort(cancel_error_server_goal.token, recovered_result)
        spin_until(
            lambda: client.result_ready(cancel_error_client_token),
            "cancel-error terminal result",
        )
        recovered_response = client.take_result_response(cancel_error_client_token)
        assert as_int8(recovered_response.status) == GoalStatus.STATUS_ABORTED

        abort_goal = cpp_types.goal()
        abort_goal.target_frame = "abort"
        abort_goal.source_frame = "base"
        abort_client_token = client.send_cpp_value(abort_goal)
        spin_until(
            lambda: client.goal_response_ready(abort_client_token),
            "abort goal response",
        )
        abort_server_goal = server.take_accepted()
        server.execute(abort_server_goal.token)
        abort_result = cpp_types.result()
        abort_result.transform.child_frame_id = "abort-result"
        server.abort(abort_server_goal.token, abort_result)
        spin_until(
            lambda: client.result_ready(abort_client_token),
            "aborted result",
        )
        aborted_response = client.take_result_response(abort_client_token)
        assert as_int8(aborted_response.status) == GoalStatus.STATUS_ABORTED
        assert str(aborted_response.result.transform.child_frame_id) == "abort-result"

        stats = server.stats()
        assert stats.goals_requested == 7
        assert stats.goals_accepted == 4
        assert stats.goals_rejected == 3
        assert stats.accepted_goals_taken == 4
        assert stats.cancel_requests == 2
        assert stats.cancels_accepted == 1
        assert stats.cancels_rejected == 1
        assert stats.execute_transitions == 3
        assert stats.feedback_published == 2
        assert stats.results_succeeded == 1
        assert stats.results_aborted == 2
        assert stats.results_canceled == 1
        assert stats.exceptions == 4
        assert stats.active_goals == 0
        assert stats.accepted_goals_ready == 0
        assert stats.python_goal_decision_crossings == 7
        assert stats.python_cancel_decision_crossings == 2
        assert stats.python_accepted_goal_crossings == 4
        assert stats.cpp_goal_shared_handoffs == 4
        assert stats.cpp_goal_id_materializations == 4
        assert stats.cpp_feedback_value_submissions == 2
        assert stats.cpp_result_value_submissions == 4
        assert stats.cpp_feedback_adapter_copies == 1
        assert stats.cpp_result_adapter_copies == 4
        assert stats.cpp_feedback_shared_handoffs == 1
        assert stats.cpp_result_shared_handoffs == 1
        assert stats.python_message_conversions == 0
        assert stats.python_serialization_calls == 0
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1

        default_server = ros.create_native_action_server(
            node, LookupTransform, "native_default_server")
        default_client = ros.create_native_action_client(
            node, LookupTransform, "native_default_server")
        assert default_client.wait_for_server(5.0)
        default_goal = cpp_types.goal()
        default_goal.target_frame = "default"
        default_client_token = default_client.send_cpp_value(default_goal)
        spin_until(
            lambda: default_client.goal_response_ready(default_client_token),
            "default goal response",
        )
        default_server_goal = default_server.take_accepted()
        default_server.execute(default_server_goal.token)
        default_result = cpp_types.result()
        default_result.transform.child_frame_id = "default-result"
        default_server.succeed(default_server_goal.token, default_result)
        spin_until(
            lambda: default_client.result_ready(default_client_token),
            "default result",
        )
        default_client.take_result_response(default_client_token)
        default_stats = default_server.stats()
        assert default_stats.python_goal_decision_crossings == 0
        assert default_stats.python_cancel_decision_crossings == 0
        assert default_stats.goals_accepted == 1
        assert default_stats.results_succeeded == 1

        close_box = {}

        def close_from_goal(_goal):
            assert not close_box["server"].close()
            return False

        close_server = ros.create_native_action_server(
            node,
            LookupTransform,
            "native_close_server",
            goal_callback=close_from_goal,
        )
        close_box["server"] = close_server
        close_client = ros.create_native_action_client(
            node, LookupTransform, "native_close_server")
        assert close_client.wait_for_server(5.0)
        close_goal = cpp_types.goal()
        close_goal.target_frame = "close"
        close_client_token = close_client.send_cpp_value(close_goal)
        spin_until(
            lambda: close_client.goal_response_ready(close_client_token),
            "deferred close goal response",
        )
        assert not close_client.goal_accepted(close_client_token)
        assert close_server.close_pending
        assert close_server.service_deferred_close()
        assert close_server.closed

        assert server.close()
        assert default_server.close()
        assert not server.close()
        assert client.close() is None
        assert default_client.close() is None
        assert close_client.close() is None
        gc.collect()
        assert str(retained[0].target_frame) == "success"
        assert type(retained[1]) is cpp_types.goal
        assert type(retained[2]) is cpp_types.goal_id
        assert type(retained[4]) is cpp_types.feedback
        assert bool(retained[4].__smartptr__())
        assert type(retained[5]) is cpp_types.result
        assert bool(retained[5].__smartptr__())
        assert str(retained[5].transform.child_frame_id) == "success-result"
        assert str(successful_response.result.transform.child_frame_id) == \
            "success-result"
        assert not server.forget(successful_server_goal.token)
        try:
            server.raw_server
        except RuntimeError as error:
            assert "closed" in str(error)
        else:
            raise AssertionError("closed server exposed its native handle")
        print("NATIVE_ACTION_SERVER_OK")

    assert server.closed
    assert default_server.closed
    assert close_server.closed
    assert str(retained[0].target_frame) == "success"
    assert type(retained[2]) is cpp_types.goal_id
    assert bool(retained[4].__smartptr__())
    assert str(retained[5].transform.child_frame_id) == "success-result"
    assert str(successful_response.result.transform.child_frame_id) == \
        "success-result"
    print("NATIVE_ACTION_SERVER_TEARDOWN_OK")


if __name__ == "__main__":
    main()
