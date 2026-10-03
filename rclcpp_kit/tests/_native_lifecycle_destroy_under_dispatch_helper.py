#!/usr/bin/env python3
"""Test PLAN-lifecycle.md S1 (risk 2/3 in the wave plan): a transition
callback is dispatched synchronously on the caller's thread
from inside trigger_transition_by_id, which itself runs on a worker thread
here. Closing (destroying) the owning NativeLifecycleNode from a second
thread while that callback is running must not crash or hang.

Mirrors the suite's established destroy-under-dispatch proofs
(_native_parameter_teardown_under_dispatch_helper.py; 6d60a85/cc70d1b/
042bb29): the registered callable's lifetime is pinned to the std::function
value itself (the reaper), independent of NativeLifecycleNode.close()
concurrently dropping this wrapper's own node_ reference. Communication
interfaces are disabled (enable_communication_interface=False) since this
test only exercises register_transition_callback, trigger_transition_by_id,
and close(). It does not need DDS services, which keeps 50 iterations fast.
"""
import faulthandler
import os
import sys
import threading

from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import CALLBACK_RETURN_SUCCESS


WATCHDOG_SECONDS = 150.0
ITERATIONS = 50


def run_iteration(session, index, pid):
    lifecycle = session.create_native_lifecycle_node(
        "lifecycle_destroy_%d_%d" % (pid, index),
        enable_communication_interface=False,
    )

    entered = threading.Event()
    release = threading.Event()
    calls = []

    def on_configure(state_id, label):
        calls.append((state_id, label))
        entered.set()
        release.wait(timeout=10.0)
        return CALLBACK_RETURN_SUCCESS

    lifecycle.register_transition_callback("configure", on_configure)
    transition_id = lifecycle.get_transition_by_label("configure")

    result_holder = {}

    def worker():
        result_holder["result"] = lifecycle.trigger_transition_by_id(
            transition_id)

    worker_thread = threading.Thread(
        target=worker, name="lifecycle-destroy-worker-%d" % index)
    worker_thread.start()
    assert entered.wait(timeout=10.0), "on_configure never entered"

    # Close the node while the transition callback above is still
    # mid-dispatch on the worker thread.
    lifecycle.close()
    assert lifecycle.closed is True

    release.set()
    worker_thread.join(timeout=15.0)
    assert not worker_thread.is_alive(), "worker thread hung"
    assert calls == [(1, "unconfigured")]
    assert result_holder.get("result") == CALLBACK_RETURN_SUCCESS


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["lifecycle-destroy-%d" % pid]) as session:
        for index in range(ITERATIONS):
            run_iteration(session, index, pid)
            print("LIFECYCLE_DESTROY_ITER_%d_OK" % index, flush=True)
    print("LIFECYCLE_DESTROY_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
