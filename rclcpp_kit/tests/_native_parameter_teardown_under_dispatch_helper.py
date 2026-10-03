#!/usr/bin/env python3
"""Test Slice 2.5a4 (PLAN-mte-unlock.md Addendum v3.2):
on/pre/post-set-parameters callbacks are worker-dispatched (rclcpp invokes
them synchronously inside set_parameters/set_parameters_atomically, called
from a worker thread or by a remote request through the node's parameter
service. Before this slice, option (a) did not cover parameter-callback
dispatch. The callable reaper from Slice 2.5a2 protects these callbacks.

The test closes the on-set-parameters bridge from within its own callback,
then drops every Python reference to the bridge and calls gc.collect(). This
exercises the premature-release window the reaper closes. The callback runs
on a worker thread calling set_parameters.
"""
import faulthandler
import gc
import os
import sys
import threading
import time
import weakref

from rclcpp_kit import direct_entities
from rclcpp_kit import native_parameters
from rclcpp_kit.native import native


WATCHDOG_SECONDS = 120.0
ITERATIONS = 50


def run_iteration(session, index, pid):
    node = session.create_node("param_teardown_%d_%d" % (pid, index))
    native_parameters.declare_parameter(
        node, native_parameters.parameter_integer("test_param", 0))

    done = threading.Event()
    holder = {}

    def on_set(_params):
        result = native_parameters.make_set_parameters_result(True, "")
        holder["cb"].close()
        holder["cb"] = None
        gc.collect()
        gc.collect()
        done.set()
        return result

    callback_ref = weakref.ref(on_set)
    holder["cb"] = native_parameters.add_on_set_parameters_callback(node, on_set)
    assert holder["cb"].callback_handoff == "compiled_python_callback"

    def worker():
        native_parameters.set_parameters(
            node, [native_parameters.parameter_integer("test_param", 1)])

    worker_thread = threading.Thread(
        target=worker, name="param-teardown-worker")
    worker_thread.start()
    worker_thread.join(timeout=15.0)
    assert not worker_thread.is_alive(), "worker thread hung"
    assert done.wait(timeout=1.0), "on_set callback never completed"
    del on_set
    gc.collect()
    direct_entities.drain_callable_reaper()
    gc.collect()
    assert callback_ref() is None, "self-removed callback remained retained"


def run_external_close(session, pid):
    node = session.create_node("param_external_close_%d" % pid)
    native_parameters.declare_parameter(
        node, native_parameters.parameter_integer("test_param", 0))

    entered = threading.Event()
    continue_callback = threading.Event()
    close_started = threading.Event()
    close_done = threading.Event()

    def on_set(_params):
        entered.set()
        assert continue_callback.wait(timeout=10.0), "callback was not released"
        return native_parameters.make_set_parameters_result(True, "")

    bridge = native_parameters.add_on_set_parameters_callback(node, on_set)
    worker_errors = []

    def worker():
        try:
            native_parameters.set_parameters_atomically(
                node, [native_parameters.parameter_integer("test_param", 1)])
        except BaseException as exc:  # noqa: BLE001
            worker_errors.append(exc)

    worker_thread = threading.Thread(
        target=worker, name="param-external-close-worker")
    worker_thread.start()
    assert entered.wait(timeout=10.0), "parameter callback was not entered"

    close_errors = []

    def closer():
        close_started.set()
        try:
            bridge.close()
        except BaseException as exc:  # noqa: BLE001
            close_errors.append(exc)
        finally:
            close_done.set()

    close_thread = threading.Thread(
        target=closer, name="param-external-close-thread")
    close_thread.start()
    assert close_started.wait(timeout=5.0), "close thread did not start"
    time.sleep(0.05)
    continue_callback.set()
    worker_thread.join(timeout=10.0)
    close_thread.join(timeout=10.0)
    assert not worker_thread.is_alive(), "parameter worker hung"
    assert not close_thread.is_alive(), "external close deadlocked"
    assert close_done.is_set(), "external close did not return"
    assert worker_errors == [], "parameter errors: %r" % (worker_errors,)
    assert close_errors == [], "close errors: %r" % (close_errors,)
    assert bridge.closed
    assert bridge.stats().to_dict() == {
        "calls": 1, "exceptions": 0, "rejections": 0}
    direct_entities.drain_callable_reaper()


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["param-teardown-%d" % pid]) as session:
        run_external_close(session, pid)
        print("PARAM_EXTERNAL_CLOSE_OK", flush=True)
        for index in range(ITERATIONS):
            run_iteration(session, index, pid)
            print("PARAM_TEARDOWN_ITER_%d_OK" % index, flush=True)
    print("PARAM_TEARDOWN_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
