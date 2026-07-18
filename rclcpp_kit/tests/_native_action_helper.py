#!/usr/bin/env python3

import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclcpp_kit.native import native
from rclcpp_kit.native_action import create_native_action_client
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_msgs.action import LookupTransform


def wait_until(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for native action state")


def main():
    python_context = Context()
    python_context.init()
    server_node = Node("native_action_server", context=python_context)
    server_group = ReentrantCallbackGroup()
    deferred_cancel_goals = []
    cancel_callbacks = []

    def goal_callback(goal):
        if goal.target_frame == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(goal_handle):
        cancel_callbacks.append(goal_handle)
        return CancelResponse.ACCEPT

    def handle_accepted(goal_handle):
        if goal_handle.request.target_frame in ("cancel", "hold"):
            deferred_cancel_goals.append(goal_handle)
        else:
            goal_handle.execute()

    def execute(goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = LookupTransform.Result()
            result.transform.child_frame_id = "canceled-result"
            return result
        for _ in range(4):
            goal_handle.publish_feedback(LookupTransform.Feedback())
        goal_handle.succeed()
        result = LookupTransform.Result()
        result.transform.child_frame_id = "stock-python-result"
        return result

    action_server = ActionServer(
        server_node,
        LookupTransform,
        "native_lookup",
        execute_callback=execute,
        goal_callback=goal_callback,
        cancel_callback=cancel_callback,
        handle_accepted_callback=handle_accepted,
        callback_group=server_group,
    )
    server_executor = MultiThreadedExecutor(
        num_threads=3, context=python_context)
    server_executor.add_node(server_node)
    server_thread = threading.Thread(target=server_executor.spin, daemon=True)
    server_thread.start()

    with native(["native-action-test"]) as ros:
        client_node = ros.create_node("native_action_client")
        client_group = ros.create_callback_group(client_node, "reentrant")
        client_executor = ros.create_executor("multi_threaded", threads=2)
        client_executor.add_node(client_node)
        executor_thread = ros.start_executor(client_executor)
        client = create_native_action_client(
            ros,
            client_node,
            LookupTransform,
            "native_lookup",
            callback_group=client_group,
            feedback_capacity=2,
        )

        assert client.wait_for_server(5.0)
        assert client.server_is_ready()
        assert client.raw_client.action_server_is_ready()

        successful = client.send_goal(LookupTransform.Goal(
            target_frame="map", source_frame="base"))
        wait_until(lambda: client.goal_response_ready(successful))
        assert client.goal_accepted(successful)
        assert client.raw_goal_handle(successful)
        wait_until(lambda: client.result_ready(successful))
        wait_until(lambda: client.stats().feedback_received >= 4)
        assert client.feedback_ready(successful)
        client.take_feedback(successful)
        client.take_feedback(successful)
        assert not client.feedback_ready(successful)
        successful_result = client.take_result(successful)
        assert successful_result.code == GoalStatus.STATUS_SUCCEEDED
        assert successful_result.result.transform.child_frame_id == \
            "stock-python-result"
        try:
            client.result_ready(successful)
        except Exception as exc:
            assert "unknown or completed" in str(exc)
        else:
            raise AssertionError("completed goal token remained usable")

        rejected_goal = client.make_goal()
        rejected_goal.target_frame = "reject"
        rejected_goal.source_frame = "base"
        rejected = client.send_goal(rejected_goal)
        wait_until(lambda: client.goal_response_ready(rejected))
        assert not client.goal_accepted(rejected)
        assert not client.raw_goal_handle(rejected)
        assert not client.result_ready(rejected)
        assert client.forget(rejected)
        assert not client.forget(rejected)

        cancel_goal = LookupTransform.Goal(
            target_frame="cancel", source_frame="base")
        canceled = client.send_goal(cancel_goal)
        wait_until(lambda: client.goal_response_ready(canceled))
        assert client.goal_accepted(canceled)
        wait_until(lambda: len(deferred_cancel_goals) == 1)
        assert client.request_cancel(canceled)
        assert not client.request_cancel(canceled)
        wait_until(lambda: client.cancel_response_ready(canceled))
        cancel_response = client.take_cancel_response(canceled)
        return_code = cancel_response.return_code
        if isinstance(return_code, str):
            return_code = ord(return_code)
        assert return_code == CancelGoal.Response.ERROR_NONE
        assert len(cancel_response.goals_canceling) == 1
        assert len(cancel_callbacks) == 1
        assert not client.cancel_response_ready(canceled)
        deferred_cancel_goals[0].execute()
        wait_until(lambda: client.result_ready(canceled))
        canceled_result = client.take_result(canceled)
        assert canceled_result.code == GoalStatus.STATUS_CANCELED
        assert canceled_result.result.transform.child_frame_id == "canceled-result"

        stats = client.stats()
        assert stats.goals_sent == 3
        assert stats.goals_accepted == 2
        assert stats.goals_rejected == 1
        assert stats.results_taken == 2
        assert stats.feedback_received >= 4
        assert stats.feedback_taken == 2
        assert stats.feedback_dropped >= 2
        assert stats.cancel_requests == 1
        assert stats.cancel_responses_taken == 1
        assert stats.forgotten == 1
        assert stats.exceptions == 0
        assert stats.active_goals == 0
        assert stats.python_goal_crossings == 3
        assert stats.python_feedback_crossings == 2
        assert stats.python_result_crossings == 2
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1
        assert executor_thread.exceptions == 0

        pending = client.send_goal(LookupTransform.Goal(
            target_frame="hold", source_frame="base"))
        wait_until(lambda: client.goal_response_ready(pending))
        assert client.goal_accepted(pending)
        wait_until(lambda: len(deferred_cancel_goals) == 2)
        assert client.stats().active_goals == 1
        client.close()
        assert client.closed
        assert client.stats().active_goals == 0
        assert client.stats().goals_sent == 4
        assert client.stats().goals_accepted == 3
        print("NATIVE_ACTION_OK")

    assert client.closed
    assert client.stats().active_goals == 0
    action_server.destroy()
    server_executor.shutdown(timeout_sec=2.0)
    server_thread.join(timeout=2.0)
    assert not server_thread.is_alive()
    server_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_ACTION_TEARDOWN_OK")


if __name__ == "__main__":
    main()
