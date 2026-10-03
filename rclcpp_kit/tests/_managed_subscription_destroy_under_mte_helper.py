#!/usr/bin/env python3
"""Test Slice 2.5a (PLAN-mte-unlock.md Addendum v2-completion):
a subscription's entity is destroyed (node removed from the executor, then
close()d) while a peer subscription's callback is running on
another native MultiThreadedExecutor worker thread. Pre-fix, this crashed
('callable was deleted' / 'terminate called without an active exception');
post-fix (ManagedSubscription), teardown drops a strong reference but keeps
the callable alive while the entity may still be referenced. The process
must exit without a crash on every iteration.

Not every iteration is guaranteed to deliver the "slow" peer's message
before its owning node is removed from the executor -- a racing node
removal can legitimately abandon a not-yet-collected message, which is a
separate teardown behavior from the use-after-free error under test. The
test requires the process to exit without a signal or hang.
"""
import faulthandler
import os
import sys
import threading
import time

from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


WATCHDOG_SECONDS = 240.0
SLOW_SLEEP_S = 0.3
ITERATIONS = 60


def run_iteration(session, cpp_type, index, pid):
    node = session.create_node("managed_destroy_mte_node_%d_%d" % (pid, index))
    executor = session.create_executor("multi_threaded", threads=2)
    executor.add_node(node)
    group = session.create_callback_group(node, "reentrant")
    qos = direct_entities.qos_from_depth(session.rclcpp, 10)

    prefix = "/direct_cpp/managed_destroy_mte/p%d/i%d" % (pid, index)
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
        # Replicates a node destroy: remove from executor, then close()
        # every subscription -- while the slow peer may still be asleep.
        executor.remove_node(node)
        slow_sub.close()
        destroyer_sub.close()
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
    time.sleep(0.1)  # settle the discovery-to-delivery gap

    spin_errors = []
    executor.spin.__release_gil__ = True

    def _spin_target():
        try:
            executor.spin()
        except BaseException as exc:  # noqa: BLE001 -- captured for the proof
            spin_errors.append(exc)

    spin_thread = threading.Thread(
        target=_spin_target, name="managed-destroy-mte-spin")
    spin_thread.start()

    message = cpp_type()
    message.data = 1
    slow_pub.publish(message)
    message2 = cpp_type()
    message2.data = 2
    destroyer_pub.publish(message2)

    slow_done.wait(timeout=15.0)  # informational only, see module docstring
    assert destroyer_done.wait(timeout=15.0), "destroyer callback never completed"

    executor.cancel()
    spin_thread.join(timeout=15.0)
    assert not spin_thread.is_alive(), "spin thread hung after teardown"
    assert spin_errors == [], "unexpected spin() exception(s): %r" % (spin_errors,)


def main():
    faulthandler.dump_traceback_later(WATCHDOG_SECONDS, file=sys.stderr, exit=True)
    pid = os.getpid()
    with native(["managed-destroy-mte-%d" % pid]) as session:
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        _, cpp_type, _ = direct_entities.resolve_supported_type(message_type)
        for index in range(ITERATIONS):
            run_iteration(session, cpp_type, index, pid)
    print("MANAGED_SUBSCRIPTION_DESTROY_UNDER_MTE_OK", flush=True)


if __name__ == "__main__":
    main()
