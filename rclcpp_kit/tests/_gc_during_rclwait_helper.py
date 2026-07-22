#!/usr/bin/env python3
"""Committed discriminating proof for Slice 2.5a2 (PLAN-mte-unlock.md
Addendum v3): the window callback-quiescence (``in_flight == 0``) cannot
observe or gate on -- a worker parked in ``rcl_wait``, holding a
wait-set-local strong copy of an entity (``dynamic_storage.hpp``), with NO
callback ever dispatched for it at all.

Unlike ``_gc_after_close_helper.py`` (which closes an in-flight peer's
subscription), this never publishes anything and never calls ``close()``
or ``destroy_subscription()`` explicitly: it creates an idle subscription,
lets a live ``MultiThreadedExecutor`` spin around it, then drops the ONLY
Python reference to the subscription facade and forces ``gc.collect()`` --
an implicit-GC teardown reachable without ANY gated destroy path. Before
Slice 2.5a2 this is exactly the residual gap the quiescence gate (option
(a) alone) could narrow but not close; the callable-lifetime reaper
(``_pinned_std_function``) ties the callable's lifetime to the
``std::function`` value itself, independent of a worker's collect timing,
so this must stay crash-free.
"""
import faulthandler
import gc
import os
import sys
import threading
import time

from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


WATCHDOG_SECONDS = 120.0
ITERATIONS = 60


def run_iteration(session, cpp_type, index, pid):
    node = session.create_node("gc_during_rclwait_node_%d_%d" % (pid, index))
    executor = session.create_executor("multi_threaded", threads=2)
    executor.add_node(node)
    qos = direct_entities.qos_from_depth(session.rclcpp, 10)

    topic = "/direct_cpp/gc_during_rclwait/p%d/i%d" % (pid, index)

    def never_called(_message):
        raise AssertionError(
            "callback must never be invoked -- this topic has no publisher")

    subscription = direct_entities.create_subscription(
        node, cpp_type, topic, never_called, qos)

    spin_errors = []
    executor.spin.__release_gil__ = True

    def _spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            spin_errors.append(exc)

    spin_thread = threading.Thread(
        target=_spin_target, name="gc-during-rclwait-spin")
    spin_thread.start()

    # Give workers a moment to actually start cycling through
    # wait_for_work/rcl_wait before striking -- no message ever arrives, so
    # every collect for this subscription upgrades its weak_ptr, finds
    # nothing to dispatch, and loops back, repeatedly, the whole time.
    time.sleep(0.01)

    # Drop the ONLY Python reference to the subscription facade -- no
    # explicit close()/destroy_subscription() at all -- and force GC, while
    # a worker may be parked in rcl_wait holding a wait-set-local strong
    # copy of the entity, no callback ever dispatched. This is the window
    # callback-quiescence (in_flight == 0) cannot observe or gate on.
    del subscription
    gc.collect()

    time.sleep(0.05)
    executor.cancel()
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after teardown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["gc-during-rclwait-%d" % pid]) as session:
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        _, cpp_type, _ = direct_entities.resolve_supported_type(message_type)
        for index in range(ITERATIONS):
            run_iteration(session, cpp_type, index, pid)
            print("GC_DURING_RCLWAIT_ITER_%d_OK" % index, flush=True)
    print("GC_DURING_RCLWAIT_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
