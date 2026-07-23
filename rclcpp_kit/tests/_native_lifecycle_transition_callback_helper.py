#!/usr/bin/env python3
"""Committed proof for PLAN-lifecycle.md S1: the transition-callback bridge
(register_transition_callback) and the Python state-machine accessors
(current_state/available_states/available_transitions/transition_graph/
get_transition_by_label/trigger_transition_by_id/trigger_transition_by_label/
initialized) on NativeLifecycleNode.

Drives a Python on_configure/on_activate/on_deactivate/on_cleanup both via a
stock rclpy client on /change_state (the native service-handler dispatch
funnel, on an executor worker thread) and via trigger_transition_by_id/label
(the synchronous, caller-thread dispatch funnel) -- both are described as
funneling through the same shim (PLAN-lifecycle.md S3.2). A second node
proves the ERROR path: a raising-equivalent (CALLBACK_RETURN_ERROR-returning)
on_configure drives native error-processing (on_error), recovering to
unconfigured -- mirroring stock rclpy's swallowed-transition-exception
contract (node.py's __execute_callback).

Every accessor's shape/values are asserted against the actual, live Jazzy
default rcl_lifecycle state/transition graph (verified once interactively
before writing these constants; this is the wave's disclosed source of
truth, not a guess).
"""
import time

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import (
    CALLBACK_RETURN_ERROR,
    CALLBACK_RETURN_SUCCESS,
)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


_EXPECTED_STATES = frozenset([
    (0, "unknown"),
    (1, "unconfigured"),
    (2, "inactive"),
    (3, "active"),
    (4, "finalized"),
    (10, "configuring"),
    (11, "cleaningup"),
    (12, "shuttingdown"),
    (13, "activating"),
    (14, "deactivating"),
    (15, "errorprocessing"),
])

_EXPECTED_TRANSITION_GRAPH = frozenset([
    (1, "configure", 1, "unconfigured", 10, "configuring"),
    (2, "cleanup", 2, "inactive", 11, "cleaningup"),
    (3, "activate", 2, "inactive", 13, "activating"),
    (4, "deactivate", 3, "active", 14, "deactivating"),
    (5, "shutdown", 1, "unconfigured", 12, "shuttingdown"),
    (6, "shutdown", 2, "inactive", 12, "shuttingdown"),
    (7, "shutdown", 3, "active", 12, "shuttingdown"),
    (10, "transition_success", 10, "configuring", 2, "inactive"),
    (11, "transition_failure", 10, "configuring", 1, "unconfigured"),
    (12, "transition_error", 10, "configuring", 15, "errorprocessing"),
    (20, "transition_success", 11, "cleaningup", 1, "unconfigured"),
    (21, "transition_failure", 11, "cleaningup", 2, "inactive"),
    (22, "transition_error", 11, "cleaningup", 15, "errorprocessing"),
    (30, "transition_success", 13, "activating", 3, "active"),
    (31, "transition_failure", 13, "activating", 2, "inactive"),
    (32, "transition_error", 13, "activating", 15, "errorprocessing"),
    (40, "transition_success", 14, "deactivating", 2, "inactive"),
    (41, "transition_failure", 14, "deactivating", 3, "active"),
    (42, "transition_error", 14, "deactivating", 15, "errorprocessing"),
    (50, "transition_success", 12, "shuttingdown", 4, "finalized"),
    (51, "transition_failure", 12, "shuttingdown", 4, "finalized"),
    (52, "transition_error", 12, "shuttingdown", 15, "errorprocessing"),
    (60, "transition_success", 15, "errorprocessing", 1, "unconfigured"),
    (61, "transition_failure", 15, "errorprocessing", 4, "finalized"),
    (62, "transition_error", 15, "errorprocessing", 4, "finalized"),
])

_EXPECTED_AVAILABLE_FROM_UNCONFIGURED = frozenset([
    (1, "configure", 1, "unconfigured", 10, "configuring"),
    (5, "shutdown", 1, "unconfigured", 12, "shuttingdown"),
])


def _call(executor, client, request, timeout_sec=5.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not future.done():
        executor.spin_once(timeout_sec=0.02)
    assert future.done()
    result = future.result()
    assert result is not None
    return result


def _change(executor, client, transition_id):
    request = ChangeState.Request()
    request.transition.id = transition_id
    response = _call(executor, client, request)
    assert response.success is True


def _state(executor, client):
    return _call(executor, client, GetState.Request()).current_state.id


def main():
    python_context = Context()
    python_context.init()
    client_node = Node(
        "native_lifecycle_transitions_client", context=python_context)
    client_executor = SingleThreadedExecutor(context=python_context)
    client_executor.add_node(client_node)

    with native(["native-lifecycle-transitions-test"]) as ros:
        lifecycle = ros.create_native_lifecycle_node(
            "managed_lifecycle_transitions")

        assert lifecycle.initialized is True
        assert set(lifecycle.available_states) == _EXPECTED_STATES
        assert set(lifecycle.transition_graph) == _EXPECTED_TRANSITION_GRAPH
        assert (
            set(lifecycle.available_transitions)
            == _EXPECTED_AVAILABLE_FROM_UNCONFIGURED
        )
        assert lifecycle.current_state == (
            State.PRIMARY_STATE_UNCONFIGURED, "unconfigured")
        assert (
            lifecycle.get_transition_by_label("configure")
            == Transition.TRANSITION_CONFIGURE
        )

        recorded = []

        def on_configure(state_id, label):
            recorded.append(("configure", state_id, label))
            return CALLBACK_RETURN_SUCCESS

        def on_activate(state_id, label):
            recorded.append(("activate", state_id, label))
            return CALLBACK_RETURN_SUCCESS

        def on_deactivate(state_id, label):
            recorded.append(("deactivate", state_id, label))
            return CALLBACK_RETURN_SUCCESS

        def on_cleanup(state_id, label):
            recorded.append(("cleanup", state_id, label))
            return CALLBACK_RETURN_SUCCESS

        lifecycle.register_transition_callback("configure", on_configure)
        lifecycle.register_transition_callback("activate", on_activate)
        lifecycle.register_transition_callback("deactivate", on_deactivate)
        lifecycle.register_transition_callback("cleanup", on_cleanup)

        executor = ros.create_executor()
        lifecycle.attach_executor(executor)
        executor_thread = ros.start_executor(executor)

        change_state = client_node.create_client(
            ChangeState, "/managed_lifecycle_transitions/change_state")
        get_state = client_node.create_client(
            GetState, "/managed_lifecycle_transitions/get_state")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if get_state.service_is_ready() and change_state.service_is_ready():
                break
            client_executor.spin_once(timeout_sec=0.05)
        assert get_state.service_is_ready()
        assert change_state.service_is_ready()

        # Drive via the stock rclpy client (/change_state): dispatch happens
        # inside the node's native service handler, on the executor worker
        # thread started above.
        _change(client_executor, change_state, Transition.TRANSITION_CONFIGURE)
        assert recorded == [("configure", 1, "unconfigured")]
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_INACTIVE
        assert lifecycle.current_state == (
            State.PRIMARY_STATE_INACTIVE, "inactive")

        # Drive via trigger_transition_by_label: the synchronous,
        # caller-thread dispatch funnel (PLAN-lifecycle.md S3.2) -- the same
        # registered callback fires through the other path.
        result = lifecycle.trigger_transition_by_label("activate")
        assert result == CALLBACK_RETURN_SUCCESS
        assert recorded[-1] == ("activate", 2, "inactive")
        assert lifecycle.current_state == (State.PRIMARY_STATE_ACTIVE, "active")
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_ACTIVE

        deactivate_id = lifecycle.get_transition_by_label("deactivate")
        result = lifecycle.trigger_transition_by_id(deactivate_id)
        assert result == CALLBACK_RETURN_SUCCESS
        assert recorded[-1] == ("deactivate", 3, "active")
        assert lifecycle.current_state == (
            State.PRIMARY_STATE_INACTIVE, "inactive")

        _change(client_executor, change_state, Transition.TRANSITION_CLEANUP)
        assert recorded[-1] == ("cleanup", 2, "inactive")
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_UNCONFIGURED
        assert lifecycle.current_state == (
            State.PRIMARY_STATE_UNCONFIGURED, "unconfigured")

        assert executor_thread.exceptions == 0

        client_node.destroy_client(change_state)
        client_node.destroy_client(get_state)
        lifecycle.close()
        assert lifecycle.closed is True

        # --- ERROR path: a second, independent node whose on_configure
        # reports CALLBACK_RETURN_ERROR must drive native error-processing
        # (the registered on_error), recovering to unconfigured -- matching
        # stock rclpy's swallowed-transition-exception contract.
        lifecycle2 = ros.create_native_lifecycle_node(
            "managed_lifecycle_error_path")
        error_recorded = []

        def on_configure_error(state_id, label):
            error_recorded.append(("configure", state_id, label))
            return CALLBACK_RETURN_ERROR

        def on_error(state_id, label):
            error_recorded.append(("error", state_id, label))
            return CALLBACK_RETURN_SUCCESS

        bridge_configure = lifecycle2.register_transition_callback(
            "configure", on_configure_error)
        bridge_error = lifecycle2.register_transition_callback(
            "error", on_error)

        configure_id = lifecycle2.get_transition_by_label("configure")
        result = lifecycle2.trigger_transition_by_id(configure_id)
        assert result == CALLBACK_RETURN_ERROR
        assert error_recorded == [
            ("configure", 1, "unconfigured"),
            ("error", 1, "unconfigured"),
        ]
        assert lifecycle2.current_state == (
            State.PRIMARY_STATE_UNCONFIGURED, "unconfigured")
        assert bridge_configure.take_exception() is None
        assert bridge_error.take_exception() is None

        lifecycle2.close()
        assert lifecycle2.closed is True
        print("NATIVE_LIFECYCLE_TRANSITIONS_OK")

    client_executor.remove_node(client_node)
    client_executor.shutdown(timeout_sec=1.0)
    client_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_LIFECYCLE_TRANSITIONS_TEARDOWN_OK")


if __name__ == "__main__":
    main()
