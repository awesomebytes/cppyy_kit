#!/usr/bin/env python3
"""Committed discriminating proof for Slice 2.5a4 (PLAN-mte-unlock.md
Addendum v3.2): the marshal-window stress applied to the parameter bridge.

``MarshalWindowHook`` (Slice 2.5a3) fires inside ``PinnedCallable::
operator()`` -- generic across every entity kind using the callable-
lifetime reaper, including the on-set-parameters bridge converted in this
slice -- right before cppyy's own marshaling/invocation of the underlying
Python callable begins.

Unlike subscriptions, a cross-thread close DURING a widened marshal window
is not just racy here, it deadlocks outright: ``set_parameters`` holds the
node's own recursive mutex for its entire synchronous dispatch, and
``close()`` (``remove_on_set_parameters_callback``) takes the same mutex --
confirmed empirically (an earlier draft of this test hung the full
watchdog). That is a *stronger* guarantee than subscriptions have, not a
gap: the bridge object's own lifetime can never race a dispatch this way.
So this exercises the window the recursive mutex does NOT rule out -- a
self-close, from the SAME thread already holding the lock (the recursive
mutex permits re-entry), widened by the hook, dropping every Python
reference and forcing gc.collect() before the dispatch call itself
returns. Must stay crash-free.
"""
import faulthandler
import gc
import os
import sys
import threading

from rclcpp_kit import direct_entities
from rclcpp_kit import native_parameters
from rclcpp_kit.native import native


WATCHDOG_SECONDS = 120.0
ITERATIONS = 50


def run_iteration(session, index, pid):
    node = session.create_node("param_marshal_window_%d_%d" % (pid, index))
    native_parameters.declare_parameter(
        node, native_parameters.parameter_integer("test_param", 0))

    entered_marshal = threading.Event()
    holder = {}

    def marshal_hook():
        entered_marshal.set()

    def on_set(_params):
        holder["cb"].close()
        holder["cb"] = None
        gc.collect()
        gc.collect()
        return native_parameters.make_set_parameters_result(True, "")

    direct_entities.set_marshal_window_hook(marshal_hook)
    try:
        holder["cb"] = native_parameters.add_on_set_parameters_callback(
            node, on_set)

        worker_errors = []

        def worker():
            try:
                native_parameters.set_parameters(
                    node, [native_parameters.parameter_integer("test_param", 1)])
            except BaseException as exc:  # noqa: BLE001 -- captured for the proof
                worker_errors.append(exc)

        worker_thread = threading.Thread(
            target=worker, name="param-marshal-window-worker")
        worker_thread.start()
        worker_thread.join(timeout=15.0)
        assert not worker_thread.is_alive(), "worker thread hung"
        assert entered_marshal.is_set(), (
            "the hook never fired -- the proof did not run"
        )
        assert worker_errors == [], (
            "unexpected set_parameters exception(s): %r" % (worker_errors,)
        )
    finally:
        direct_entities.clear_marshal_window_hook()


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["param-marshal-window-%d" % pid]) as session:
        for index in range(ITERATIONS):
            run_iteration(session, index, pid)
            print("PARAM_MARSHAL_WINDOW_ITER_%d_OK" % index, flush=True)
    print("PARAM_MARSHAL_WINDOW_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
