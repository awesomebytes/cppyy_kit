#!/usr/bin/env python3
"""Test the action server lifetime behavior from Slice 2.5a
(PLAN-mte-unlock.md Addendum v2-completion). The dispatch model keeps these
entities safe under a live MultiThreadedExecutor, so they do not use the
ManagedCallbackEntityImpl lifetime fix.

Two mechanisms already make this safe, independent of Slice 2.5a:

  * ``_DispatchState`` (native_action_server.py) rejects a goal/cancel
    dispatch outright unless it runs on the thread that created the action
    server (its P0 "creator-thread spinning" guard), so a Python callback
    crossing never races a native MultiThreadedExecutor worker thread the
    way a raw subscription/timer callback could pre-Slice-2.5a.
  * ``NativeActionServer.close()`` defers to ``close_pending`` instead of
    tearing the callback down while ``_dispatch_state.depth`` is nonzero,
    and only ``service_deferred_close()`` -- polled by whatever pumps the
    executor, never the native wait set -- finishes the close once the
    in-flight callback has returned.

This test destroys (``close()``) an action server from a second thread while
its goal_callback is waiting,
under a real ``multi_threaded`` executor. It must defer rather than crash,
and the deferred close must flush cleanly once the callback completes.
"""
import faulthandler
import os
import sys
import threading
import time

import cppyy

from rclcpp_kit.native import native
from rclcpp_kit.native_action import resolve_cpp_action_type
from tf2_msgs.action import LookupTransform


WATCHDOG_SECONDS = 60.0
CALLBACK_SLEEP_S = 0.3


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    duration = cppyy.gbl.std.chrono.milliseconds(10)

    with native(["native-action-destroy-mte-%d" % pid]) as ros:
        cpp_types = resolve_cpp_action_type(LookupTransform)
        node = ros.create_node("native_action_destroy_mte_node_%d" % pid)
        executor = ros.create_executor("multi_threaded", threads=2)
        executor.add_node(node)
        group = ros.create_callback_group(node, "mutually_exclusive")

        callback_started = threading.Event()
        close_returned = threading.Event()
        close_results = []

        def goal_callback(_goal):
            callback_started.set()
            time.sleep(CALLBACK_SLEEP_S)
            return True

        server = ros.create_native_action_server(
            node, LookupTransform, "native_action_destroy_mte",
            goal_callback=goal_callback, callback_group=group)
        client = ros.create_native_action_client(
            node, LookupTransform, "native_action_destroy_mte")
        assert client.wait_for_server(5.0)

        def destroyer():
            assert callback_started.wait(timeout=15.0), (
                "goal_callback never started")
            close_results.append(server.close())
            close_returned.set()

        destroyer_thread = threading.Thread(
            target=destroyer, name="native-action-destroy-mte-destroyer")
        destroyer_thread.start()

        goal = cpp_types.goal()
        goal.target_frame = "destroy-under-mte"
        goal.source_frame = "base"
        token = client.send_cpp_value(goal)

        deadline = time.monotonic() + 15.0
        while not close_returned.is_set() and time.monotonic() < deadline:
            executor.spin_once(duration)
        assert close_returned.is_set(), "destroyer thread never observed close()"
        destroyer_thread.join(timeout=15.0)
        assert not destroyer_thread.is_alive(), "destroyer thread hung"

        # close() raced the in-flight goal_callback -- it must have deferred
        # (False) instead of severing the callback while depth was nonzero.
        assert close_results == [False], (
            "close() must defer while goal_callback is in flight, got %r" %
            (close_results,)
        )
        assert server.close_pending

        response_deadline = time.monotonic() + 15.0
        while (
            not client.goal_response_ready(token)
            and time.monotonic() < response_deadline
        ):
            executor.spin_once(duration)
        assert client.goal_response_ready(token), "goal response never arrived"

        # Flush the deferred close, exactly like a pump loop would on its
        # next iteration once the in-flight callback has returned.
        flush_deadline = time.monotonic() + 15.0
        while (
            not server.service_deferred_close()
            and time.monotonic() < flush_deadline
        ):
            executor.spin_once(duration)
        assert server.closed, "deferred close never flushed"

        client.close()

    print("NATIVE_ACTION_SERVER_DESTROY_UNDER_MTE_OK", flush=True)


if __name__ == "__main__":
    main()
