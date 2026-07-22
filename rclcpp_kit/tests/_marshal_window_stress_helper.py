#!/usr/bin/env python3
"""Committed discriminating proof for Slice 2.5a2's reaper mechanism
(PLAN-mte-unlock.md Addendum v3.1): the marshal window.

A callback-quiescence counter (the product's ``_quiesce_or_raise``/
in-flight counter) only increments once a callback has actually entered
the containment shim. A worker that has already obtained the executable
and committed to dispatch -- cppyy mid-marshal, converting the C++
message into Python arguments, the shim not yet entered -- is invisible
to that counter by construction. It reads ``in_flight == 0`` and a
concurrent destroy proceeds to sever the entity, racing a worker that is
about to invoke the callable. This window is not stress-testable from
pure Python (there is no Python-level vantage point on cppyy's own
marshaling step), so this uses ``direct_entities.set_marshal_window_hook``
-- a test-only, default-off suite hook -- to widen it on demand and land a
concurrent destroy reliably inside it.

Before Slice 2.5a2 (the callable-lifetime reaper), this window would sever
the Python callable while a worker holds the ``AnyExecutable`` pinning the
entity and its stored ``std::function`` -- exactly the "callable was
deleted" class. The reaper ties the callable's lifetime to the
``std::function`` copy itself (independent of any Python wrapper's GC
timing), so this must stay crash-free regardless of when, relative to the
marshal window, the destroy happens.
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
HOOK_SLEEP_S = 0.2
ITERATIONS = 60


def run_iteration(session, cpp_type, index, pid):
    node = session.create_node("marshal_window_node_%d_%d" % (pid, index))
    executor = session.create_executor("multi_threaded", threads=2)
    executor.add_node(node)
    qos = direct_entities.qos_from_depth(session.rclcpp, 10)

    topic = "/direct_cpp/marshal_window/p%d/i%d" % (pid, index)
    publisher = direct_entities.create_publisher(node, cpp_type, topic, qos)

    entered_marshal = threading.Event()
    proceed = threading.Event()
    received = []

    def marshal_hook():
        # Fires INSIDE PinnedCallable::operator(), before cppyy's own
        # marshaling/invocation of the Python callback begins -- i.e. the
        # worker has already obtained the executable and committed to
        # dispatch, but has not yet reached the shim (which would bump the
        # product's in-flight counter). Widen that window on demand.
        entered_marshal.set()
        proceed.wait(timeout=15.0)

    direct_entities.set_marshal_window_hook(marshal_hook)
    try:
        subscription = direct_entities.create_subscription(
            node, cpp_type, topic, received.append, qos)

        deadline = time.monotonic() + 15.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert publisher.get_subscription_count() == 1
        time.sleep(0.1)  # settle the discovery-to-delivery gap

        spin_errors = []
        executor.spin.__release_gil__ = True

        def _spin_target():
            try:
                executor.spin()
            except BaseException as exc:  # noqa: BLE001 -- captured for the proof
                spin_errors.append(exc)

        spin_thread = threading.Thread(
            target=_spin_target, name="marshal-window-spin")
        spin_thread.start()

        message = cpp_type()
        message.data = 1
        publisher.publish(message)

        # Wait until a worker is genuinely parked inside the marshal
        # window (mid-dispatch, pre-shim) before striking.
        assert entered_marshal.wait(timeout=15.0), (
            "worker never entered the marshal window -- the hook did not "
            "fire, the proof did not run"
        )

        # Sever the entity WHILE that worker is committed to invoking its
        # callable but has not yet done so. Nothing at the suite level
        # gates this (there is no in-flight counter here) -- the point is
        # that the reaper alone must make this safe regardless. Also drop
        # every Python-level reference to the subscription facade (hence
        # to `managed`) and force a GC pass -- matching gc_after_close's
        # pattern -- so this actually exercises "the wrapper's own
        # keep-alive protection is gone" rather than accidentally staying
        # safe only because a still-live local variable happens to keep
        # `managed` referenced throughout.
        subscription.close()
        subscription = None
        gc.collect()

        # Let the parked worker continue: it will now actually attempt the
        # marshal + invoke the callback of the just-closed subscription.
        proceed.set()

        # Give it every chance to actually happen before tearing down.
        time.sleep(HOOK_SLEEP_S + 0.5)

        executor.cancel()
        spin_thread.join(timeout=15.0)
        assert not spin_thread.is_alive(), "spin thread hung after teardown"
        assert spin_errors == [], (
            "unexpected spin() exception(s): %r" % (spin_errors,)
        )
    finally:
        direct_entities.clear_marshal_window_hook()


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["marshal-window-%d" % pid]) as session:
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        _, cpp_type, _ = direct_entities.resolve_supported_type(message_type)
        for index in range(ITERATIONS):
            run_iteration(session, cpp_type, index, pid)
            print("MARSHAL_WINDOW_ITER_%d_OK" % index, flush=True)
    print("MARSHAL_WINDOW_ALL_OK", flush=True)


if __name__ == "__main__":
    main()
