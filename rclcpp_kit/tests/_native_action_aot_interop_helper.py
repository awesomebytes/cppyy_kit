#!/usr/bin/env python3

import os
from pathlib import Path
import select
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from rclcpp_kit.native import native
from rclcpp_kit.native_action import resolve_cpp_action_type
from tf2_msgs.action import LookupTransform


RESULT_TIMEOUT_S = 15.0
PEER_TIMEOUT_S = 30.0


def wait_until(predicate, description):
    deadline = time.monotonic() + RESULT_TIMEOUT_S
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError(f"timed out waiting for {description}")


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: helper.py PATH_TO_AOT_PEER")
    domain = os.environ.get("ROS_DOMAIN_ID")
    assert domain, "the parent test must assign an isolated ROS_DOMAIN_ID"
    peer_path = Path(sys.argv[1]).resolve()
    assert peer_path.is_file() and os.access(peer_path, os.X_OK), peer_path
    action_name = f"/rclcpp_kit/aot_action_d{domain}_p{os.getpid()}"

    peer = subprocess.Popen(
        [str(peer_path), action_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=os.environ.copy(),
    )
    peer_stdout = ""
    peer_stderr = ""
    try:
        readable, _, _ = select.select([peer.stdout], [], [], 10.0)
        assert readable, "AOT action server did not report readiness within 10s"
        ready = peer.stdout.readline().strip()
        peer_stdout = ready + "\n"
        assert ready == f"AOT_ACTION_SERVER_READY {action_name}", ready

        with native(["native-action-aot-interop"]) as ros:
            cpp_types = resolve_cpp_action_type(LookupTransform)
            node = ros.create_node(f"managed_action_client_d{domain}")
            executor = ros.create_executor("multi_threaded", threads=2)
            executor.add_node(node)
            executor_thread = ros.start_executor(executor)
            client = ros.create_native_action_client(
                node,
                LookupTransform,
                action_name,
                feedback_capacity=3,
            )
            assert client.wait_for_server(10.0)

            goal = client.make_goal()
            goal.target_frame = "aot-target-314159"
            goal.source_frame = "managed-source-271828"
            goal.timeout.sec = 1
            goal.timeout.nanosec = 234567890
            goal.advanced = False
            assert type(goal) is cpp_types.goal
            token = client.send_cpp_value(goal)

            wait_until(
                lambda: client.goal_response_ready(token), "goal response")
            assert client.goal_accepted(token)
            assert client.raw_goal_handle(token)
            goal_id = client.goal_id(token)
            goal_response = client.goal_response(token)
            assert type(goal_id) is cpp_types.goal_id
            assert any(int(value) for value in goal_id.uuid)
            assert type(goal_response) is cpp_types.goal_response
            assert goal_response.accepted is True
            wait_until(
                lambda: client.stats().feedback_received == 3,
                "three feedback callbacks",
            )
            feedback = [client.take_feedback_message(token) for _ in range(3)]
            assert all(item is not None for item in feedback)
            assert all(type(item) is cpp_types.feedback_message for item in feedback)
            assert all(
                list(item.goal_id.uuid) == list(goal_id.uuid)
                for item in feedback
            )
            assert not client.feedback_ready(token)
            wait_until(lambda: client.result_ready(token), "terminal result")
            action_result = client.take_result_response(token)
            assert type(action_result) is cpp_types.result_response
            result_status = action_result.status
            if isinstance(result_status, str):
                result_status = ord(result_status)
            assert result_status == GoalStatus.STATUS_SUCCEEDED
            result = action_result.result
            assert result.transform.header.frame_id == "aot-result-frame-161803"
            assert result.transform.child_frame_id == "aot-result-child-141421"
            assert result.transform.transform.translation.x == 31.4159
            assert result.transform.transform.translation.y == 27.1828
            assert result.transform.transform.translation.z == 1.61803
            assert result.transform.transform.rotation.w == 1.0
            error_code = result.error.error
            if isinstance(error_code, str):
                error_code = ord(error_code)
            assert error_code == 0
            assert result.error.error_string == "aot-action-result-173205"

            stats = client.stats()
            assert stats.goals_sent == 1
            assert stats.goals_accepted == 1
            assert stats.goals_rejected == 0
            assert stats.results_taken == 1
            assert stats.feedback_received == 3
            assert stats.feedback_taken == 3
            assert stats.feedback_dropped == 0
            assert stats.cancel_requests == 0
            assert stats.cancel_responses_taken == 0
            assert stats.forgotten == 0
            assert stats.exceptions == 0
            assert stats.active_goals == 0
            assert stats.python_goal_crossings == 1
            assert stats.python_feedback_crossings == 3
            assert stats.python_result_crossings == 1
            assert stats.cpp_goal_value_submissions == 1
            assert stats.cpp_goal_id_materializations == 1
            assert stats.cpp_goal_response_materializations == 1
            assert stats.cpp_feedback_message_materializations == 3
            assert stats.cpp_result_response_materializations == 1
            assert stats.compile_cache_hits + stats.compile_cache_misses == 1
            assert executor_thread.exceptions == 0
        assert client.closed
        assert executor_thread.closed
        assert result.error.error_string == "aot-action-result-173205"
        assert list(feedback[0].goal_id.uuid) == list(goal_id.uuid)

        remaining_stdout, peer_stderr = peer.communicate(timeout=PEER_TIMEOUT_S)
        peer_stdout += remaining_stdout
    except Exception:
        if peer.poll() is None:
            peer.kill()
        remaining_stdout, peer_stderr = peer.communicate()
        peer_stdout += remaining_stdout
        raise

    assert peer.returncode == 0, (
        f"peer exit={peer.returncode}\n"
        f"stdout:\n{peer_stdout}\nstderr:\n{peer_stderr}"
    )
    expected = (
        "AOT_ACTION_SERVER_OK "
        "goal=aot-target-314159:managed-source-271828 "
        "feedback=3 result=aot-action-result-173205"
    )
    assert expected in peer_stdout, peer_stdout
    assert "AOT_ACTION_PEER_TEARDOWN_OK" in peer_stdout, peer_stdout
    print(f"NATIVE_ACTION_AOT_INTEROP_OK domain={domain}")
    print("NATIVE_ACTION_AOT_TEARDOWN_OK")


if __name__ == "__main__":
    main()
