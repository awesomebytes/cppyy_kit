#!/usr/bin/env python3
"""Live native guard-condition wake and wait-set proof."""

import time

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
from rclcpp_kit.native import native


bringup_rclcpp()
cppyy.cppdef(
    r"""
    #include <chrono>
    #include <memory>
    #include <thread>
    #include <rclcpp/rclcpp.hpp>

    namespace waitset_proof {
    void trigger_after(std::shared_ptr<rclcpp::GuardCondition> gc, long delay_ns) {
      std::thread([gc, delay_ns]() {
        std::this_thread::sleep_for(std::chrono::nanoseconds(delay_ns));
        gc->trigger();
      }).detach();
    }
    }  // namespace waitset_proof
    """
)


guard = None
wait_set = None
with native(["native-waitset-proof"]) as session:
    guard = session.create_native_guard_condition()
    wait_set = session.create_native_wait_set()

    raw_guard = guard.raw_guard_condition
    assert type(raw_guard) is cppyy.gbl.rclcpp.GuardCondition
    assert guard.address == cppyy.addressof(raw_guard)
    del raw_guard
    assert type(wait_set.raw_wait_set) is cppyy.gbl.rclcpp.WaitSet
    print("NATIVE_WAITSET_IDENTITY_OK")

    wait_set.add_guard_condition(guard)
    guard.trigger()
    assert wait_set.wait(0) == "ready"
    print("NATIVE_WAITSET_LATCH_OK")

    thread_guard = session.create_native_guard_condition()
    thread_wait_set = session.create_native_wait_set()
    thread_wait_set.add_guard_condition(thread_guard)
    cppyy.gbl.waitset_proof.trigger_after(
        thread_guard.raw_guard_condition, 50_000_000)
    start = time.monotonic()
    result = thread_wait_set.wait(5_000_000_000)
    elapsed = time.monotonic() - start
    assert result == "ready", "expected cross-thread trigger to wake the wait set"
    assert elapsed < 2.0, "cross-thread wake took too long: %.3fs" % elapsed
    print("NATIVE_WAITSET_CROSS_THREAD_WAKE_OK")

    second_wait_set = session.create_native_wait_set()
    try:
        second_wait_set.add_guard_condition(guard)
    except Exception:
        pass
    else:
        raise AssertionError(
            "adding an in-use guard condition to a second wait set did not raise")
    print("NATIVE_WAITSET_EXCLUSIVE_OK")

    timeout_guard = session.create_native_guard_condition()
    timeout_wait_set = session.create_native_wait_set()
    timeout_wait_set.add_guard_condition(timeout_guard)
    assert timeout_wait_set.wait(50_000_000) == "timeout"
    print("NATIVE_WAITSET_TIMEOUT_OK")

assert guard.closed
assert wait_set.closed
for accessor in (
    guard.trigger,
    lambda: guard.raw_guard_condition,
    lambda: guard.address,
):
    try:
        accessor()
    except RuntimeError as exception:
        assert "closed" in str(exception)
    else:
        raise AssertionError("closed guard condition allowed access after teardown")
for accessor in (
    lambda: wait_set.wait(),
    lambda: wait_set.raw_wait_set,
    lambda: wait_set.address,
):
    try:
        accessor()
    except RuntimeError as exception:
        assert "closed" in str(exception)
    else:
        raise AssertionError("closed wait set allowed access after teardown")
print("NATIVE_WAITSET_LIFECYCLE_OK")
