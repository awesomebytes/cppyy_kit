#!/usr/bin/env python3

import time

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import create_native_lifecycle_node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


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
    client_node = Node("native_lifecycle_client", context=python_context)
    client_executor = SingleThreadedExecutor(context=python_context)
    client_executor.add_node(client_node)

    with native(["native-lifecycle-test"]) as ros:
        lifecycle = create_native_lifecycle_node(ros, "managed_lifecycle")
        assert lifecycle in ros.resources
        raw_node = lifecycle.raw_node
        assert int(raw_node.get_current_state().id()) == (
            State.PRIMARY_STATE_UNCONFIGURED)
        assert str(raw_node.get_current_state().label()) == "unconfigured"
        assert str(raw_node.get_name()) == "managed_lifecycle"
        del raw_node

        executor = ros.create_executor()
        lifecycle.attach_executor(executor)
        assert lifecycle.attached_executors == 1
        executor_thread = ros.start_executor(executor)

        get_state = client_node.create_client(
            GetState, "/managed_lifecycle/get_state")
        change_state = client_node.create_client(
            ChangeState, "/managed_lifecycle/change_state")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if get_state.service_is_ready() and change_state.service_is_ready():
                break
            client_executor.spin_once(timeout_sec=0.05)
        assert get_state.service_is_ready()
        assert change_state.service_is_ready()

        assert _state(client_executor, get_state) == State.PRIMARY_STATE_UNCONFIGURED
        _change(
            client_executor,
            change_state,
            Transition.TRANSITION_CONFIGURE,
        )
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_INACTIVE
        _change(
            client_executor,
            change_state,
            Transition.TRANSITION_ACTIVATE,
        )
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_ACTIVE
        assert str(lifecycle.raw_node.get_current_state().label()) == "active"
        _change(
            client_executor,
            change_state,
            Transition.TRANSITION_DEACTIVATE,
        )
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_INACTIVE
        _change(
            client_executor,
            change_state,
            Transition.TRANSITION_CLEANUP,
        )
        assert _state(client_executor, get_state) == State.PRIMARY_STATE_UNCONFIGURED
        assert executor_thread.exceptions == 0

        lifecycle.close()
        assert lifecycle.closed is True
        assert lifecycle.attached_executors == 0
        print("NATIVE_LIFECYCLE_OK")

    assert lifecycle.closed is True
    client_node.destroy_client(get_state)
    client_node.destroy_client(change_state)
    client_executor.remove_node(client_node)
    client_executor.shutdown(timeout_sec=1.0)
    client_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_LIFECYCLE_TEARDOWN_OK")


if __name__ == "__main__":
    main()
