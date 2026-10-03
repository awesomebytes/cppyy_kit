import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_action_server import (
    _DispatchState,
    NativeActionServer,
    create_native_action_server,
)


class _ClosedImplementation:
    def close(self):
        pass


class _CppTypes:
    class feedback:
        pass

    class result:
        pass


def _server():
    return NativeActionServer(
        _ClosedImplementation(),
        _CppTypes,
        _DispatchState(None, None),
        None,
        None,
        object(),
        object(),
        "source",
        {"cached": False},
    )


def test_closed_action_server_rejects_work_without_touching_cpp():
    server = _server()
    assert server.close()
    assert not server.close()
    assert server.closed
    assert not server.close_pending
    assert not server.forget(1)
    for operation in (
        lambda: server.raw_server,
        server.accepted_ready_count,
        server.take_accepted,
        lambda: server.status(1),
        lambda: server.is_active(1),
        lambda: server.is_canceling(1),
        lambda: server.is_executing(1),
        lambda: server.execute(1),
        server.make_feedback_shared,
        server.make_result_shared,
        lambda: server.publish_feedback(1, _CppTypes.feedback()),
        lambda: server.publish_feedback_shared(1, _CppTypes.feedback()),
        lambda: server.succeed(1, _CppTypes.result()),
        lambda: server.succeed_shared(1, _CppTypes.result()),
        lambda: server.abort(1, _CppTypes.result()),
        lambda: server.abort_shared(1, _CppTypes.result()),
        lambda: server.canceled(1, _CppTypes.result()),
        lambda: server.canceled_shared(1, _CppTypes.result()),
    ):
        with pytest.raises(RuntimeError, match="closed"):
            operation()


@pytest.mark.parametrize(
    ("options", "exception", "match"),
    (
        ({"action_name": "  "}, ValueError, "action_name"),
        ({"goal_callback": object()}, TypeError, "goal_callback"),
        ({"cancel_callback": object()}, TypeError, "cancel_callback"),
        ({"result_timeout": -1}, ValueError, "result_timeout"),
        ({"result_timeout": ((1 << 63) - 1) / 1e9}, ValueError, "int64"),
        ({"result_timeout": 1e100}, ValueError, "int64"),
        ({"goal_service_qos": object()}, ValueError, "all five"),
    ),
)
def test_invalid_options_are_rejected_before_compilation(options, exception, match):
    arguments = {
        "owner": None,
        "node": None,
        "action_type": object,
        "action_name": "action",
    }
    arguments.update(options)
    with pytest.raises(exception, match=match):
        create_native_action_server(**arguments)


def test_coroutine_decision_callbacks_fail_closed_before_compilation():
    async def decide(_value):
        return True

    with pytest.raises(TypeError, match="goal_callback must be synchronous"):
        create_native_action_server(
            None, None, object, "action", goal_callback=decide)
    with pytest.raises(TypeError, match="cancel_callback must be synchronous"):
        create_native_action_server(
            None, None, object, "action", cancel_callback=decide)


def test_non_action_type_is_rejected_before_native_compilation():
    with pytest.raises(TypeError, match="ROS action class"):
        create_native_action_server(None, None, object, "action")


def test_native_action_server_live_exact_cpp_interop():
    proc = run_helper("_native_action_server_helper.py", timeout=360)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_ACTION_SERVER_OK" in proc.stdout
    assert "NATIVE_ACTION_SERVER_TEARDOWN_OK" in proc.stdout


def test_native_action_server_destroy_under_live_mte_does_not_crash():
    """Slice 2.5a test (PLAN-mte-unlock.md Addendum v2-completion):
    action servers are safe under a live MultiThreadedExecutor according to the
    dispatch model alone (creator-thread-only decision dispatch, plus an
    existing depth-tracked deferred close) and need no ManagedCallbackEntity
    lifetime fix. Destroying one while its goal_callback is running must
    defer destruction and avoid a crash."""
    proc = run_helper(
        "_native_action_server_destroy_under_mte_helper.py", timeout=90)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_ACTION_SERVER_DESTROY_UNDER_MTE_OK" in proc.stdout, (
        format_output(proc)
    )


def test_native_action_server_cold_and_warm_cache(tmp_path):
    helper = Path(__file__).with_name("_native_action_server_cache_helper.py")
    env = os.environ.copy()
    env["XDG_CACHE_HOME"] = str(tmp_path / "cache")

    def run():
        process = subprocess.run(
            [sys.executable, str(helper)],
            capture_output=True,
            text=True,
            timeout=300,
            check=False,
            env=env,
        )
        assert process.returncode == 0, format_output(process)
        return json.loads(process.stdout.splitlines()[-1])

    cold = run()
    warm = run()
    assert cold["first_cached"] is False
    assert cold["second_cached"] is True
    assert warm["first_cached"] is True
    assert warm["second_cached"] is True
    assert cold["source_ids"][0] == cold["source_ids"][1]
    assert warm["source_ids"] == cold["source_ids"]
    assert warm["shared_objects"] == cold["shared_objects"]
    assert len(set(cold["shared_objects"])) == 1
