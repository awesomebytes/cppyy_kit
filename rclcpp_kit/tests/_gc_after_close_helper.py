#!/usr/bin/env python3
"""Committed proof for Slice 2.5a2 (PLAN-mte-unlock.md Addendum v3): does
explicitly dropping ALL Python references to a subscription (and its
``managed`` wrapper) and forcing ``gc.collect()`` IMMEDIATELY after
``close()``, while a peer callback is genuinely in flight, reproduce the
UAF that ``ManagedCallbackEntityImpl`` (Slice 2.5a) alone did not fully
close?

This is exactly the probe that found Slice 2.5a incomplete: pre-2.5a2 this
crashed 11/60 with the original ``callable was deleted`` signature, because
``cppyy_kit.keep_alive`` pinned the callable only on a Python wrapper
object whose GC timing is not bound to the native entity. Post-2.5a2 (the
callable-lifetime reaper, ``_pinned_std_function``), the callable's
lifetime is bound to the ``std::function`` value itself -- unconditionally,
regardless of the wrapper's GC timing -- so this must now stay crash-free.
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
SLOW_SLEEP_S = 0.3
ITERATIONS = 60


def run_iteration(session, cpp_type, index, pid):
    node = session.create_node("gc_after_close_node_%d_%d" % (pid, index))
    executor = session.create_executor("multi_threaded", threads=2)
    executor.add_node(node)
    group = session.create_callback_group(node, "reentrant")
    qos = direct_entities.qos_from_depth(session.rclcpp, 10)

    prefix = "/direct_cpp/gc_after_close/p%d/i%d" % (pid, index)
    slow_topic = prefix + "/slow"
    destroyer_topic = prefix + "/destroyer"
    slow_pub = direct_entities.create_publisher(node, cpp_type, slow_topic, qos)
    destroyer_pub = direct_entities.create_publisher(
        node, cpp_type, destroyer_topic, qos)

    slow_done = threading.Event()
    destroyer_done = threading.Event()

    def slow_callback(_message):
        time.sleep(SLOW_SLEEP_S)
        slow_done.set()

    def destroyer_callback(_message):
        # Close the slow peer's subscription, then AGGRESSIVELY drop every
        # Python-level reference to it (including the node's own list entry)
        # and force a GC pass -- immediately, while the slow callback is
        # genuinely still asleep inside Python on the other worker.
        nonlocal slow_sub
        slow_sub.close()
        slow_sub = None
        destroyer_sub.close()
        gc.collect()
        gc.collect()
        destroyer_done.set()

    slow_sub = direct_entities.create_subscription(
        node, cpp_type, slow_topic, slow_callback, qos, callback_group=group)
    destroyer_sub = direct_entities.create_subscription(
        node, cpp_type, destroyer_topic, destroyer_callback, qos,
        callback_group=group)

    deadline = time.monotonic() + 15.0
    while (
        (
            slow_pub.get_subscription_count() < 1
            or destroyer_pub.get_subscription_count() < 1
        )
        and time.monotonic() < deadline
    ):
        time.sleep(0.01)
    assert slow_pub.get_subscription_count() == 1
    assert destroyer_pub.get_subscription_count() == 1
    time.sleep(0.1)

    spin_errors = []
    executor.spin.__release_gil__ = True

    def _spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            spin_errors.append(exc)

    spin_thread = threading.Thread(target=_spin_target, name="gc-after-close-spin")
    spin_thread.start()

    message = cpp_type()
    message.data = 1
    slow_pub.publish(message)
    message2 = cpp_type()
    message2.data = 2
    destroyer_pub.publish(message2)

    slow_done.wait(timeout=15.0)  # informational only
    assert destroyer_done.wait(timeout=15.0), "destroyer callback never completed"

    # Give the (already-severed-from-Python, GC'd) slow subscription's
    # in-flight rclcpp-side dispatch every chance to actually happen: sleep
    # past when the slow callback would have returned and a worker would
    # loop back to collect/redispatch.
    time.sleep(SLOW_SLEEP_S + 0.5)

    executor.cancel()
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after teardown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["gc-after-close-%d" % pid]) as session:
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        _, cpp_type, _ = direct_entities.resolve_supported_type(message_type)
        for index in range(ITERATIONS):
            run_iteration(session, cpp_type, index, pid)
            print("GC_AFTER_CLOSE_ITER_%d_OK" % index, flush=True)
    print("GC_AFTER_CLOSE_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
