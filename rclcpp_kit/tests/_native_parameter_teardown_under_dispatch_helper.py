#!/usr/bin/env python3
"""Committed proof for Slice 2.5a4 (PLAN-mte-unlock.md Addendum v3.2):
on/pre/post-set-parameters callbacks are worker-dispatched (rclcpp invokes
them synchronously inside set_parameters/set_parameters_atomically, from a
worker thread calling node.set_parameters(...) directly or from the node's
parameter service on a remote request) and, before this slice, were NOT
covered by option (a) (the product's in-flight counter does not see
parameter-callback dispatch at all) -- only the native-owned-callable-
lifetime treatment (the reaper, Slice 2.5a2) protects them.

This closes the on-set-parameters bridge from WITHIN its own callback
(a self-teardown, analogous to the subscription/timer/service self-destroy
proofs), then drops every Python reference to the bridge and forces
gc.collect() -- exercising exactly the premature-release window the
reaper closes, on a worker thread genuinely dispatching through
set_parameters.
"""
import faulthandler
import gc
import os
import sys
import threading

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

    holder["cb"] = native_parameters.add_on_set_parameters_callback(node, on_set)

    def worker():
        native_parameters.set_parameters(
            node, [native_parameters.parameter_integer("test_param", 1)])

    worker_thread = threading.Thread(
        target=worker, name="param-teardown-worker")
    worker_thread.start()
    worker_thread.join(timeout=15.0)
    assert not worker_thread.is_alive(), "worker thread hung"
    assert done.wait(timeout=1.0), "on_set callback never completed"


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["param-teardown-%d" % pid]) as session:
        for index in range(ITERATIONS):
            run_iteration(session, index, pid)
            print("PARAM_TEARDOWN_ITER_%d_OK" % index, flush=True)
    print("PARAM_TEARDOWN_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
