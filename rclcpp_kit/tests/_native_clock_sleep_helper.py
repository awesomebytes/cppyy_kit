#!/usr/bin/env python3
"""Live wall-time, identity, sim-time, and interrupt proof for clock sleep."""

import time

import cppyy

from rclcpp_kit import direct_entities
from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


bringup_rclcpp()
cppyy.cppdef(
    r"""
    #include <atomic>
    #include <chrono>
    #include <cstdint>
    #include <memory>
    #include <thread>
    #include <rclcpp/rclcpp.hpp>

    namespace clock_sleep_proof {
    class SleepOutcome {
    public:
      bool done() const { return done_.load(std::memory_order_acquire); }
      bool result() const { return result_.load(std::memory_order_acquire); }
      void set(bool value)
      {
        result_.store(value, std::memory_order_release);
        done_.store(true, std::memory_order_release);
      }
    private:
      std::atomic<bool> done_{false};
      std::atomic<bool> result_{false};
    };

    std::shared_ptr<SleepOutcome> make_outcome()
    {
      return std::make_shared<SleepOutcome>();
    }

    void sleep_until_after(
        std::shared_ptr<rclcpp::Clock> clock,
        std::shared_ptr<rclcpp::Context> context,
        int64_t nanoseconds,
        std::shared_ptr<SleepOutcome> outcome)
    {
      std::thread([clock, context, nanoseconds, outcome]() {
        rclcpp::Time target(nanoseconds, clock->get_clock_type());
        outcome->set(clock->sleep_until(target, context));
      }).detach();
    }

    void sleep_for_after(
        std::shared_ptr<rclcpp::Clock> clock,
        std::shared_ptr<rclcpp::Context> context,
        int64_t duration_ns,
        std::shared_ptr<SleepOutcome> outcome)
    {
      std::thread([clock, context, duration_ns, outcome]() {
        outcome->set(clock->sleep_for(
          rclcpp::Duration(std::chrono::nanoseconds(duration_ns)), context));
      }).detach();
    }
    }  // namespace clock_sleep_proof
    """
)


with native(["native-clock-sleep-proof"]) as session:
    # 1. Wall-time sleep_for.
    node = session.create_node("native_clock_sleep")
    sleeper = session.create_native_clock_sleeper(node)
    start = time.monotonic()
    result = sleeper.sleep_for(50_000_000)
    elapsed = time.monotonic() - start
    assert result is True
    assert elapsed >= 0.045, "sleep_for returned too early: %.4fs" % elapsed
    print("NATIVE_CLOCK_SLEEP_WALL_OK")

    # 2. Clock identity -- sleeps on the exact clock the clock foundation retains.
    clock = session.create_native_node_clock(node)
    assert sleeper.clock_address == clock.address
    print("NATIVE_CLOCK_SLEEP_IDENTITY_OK")

    # 2b. Wrapper-level sleep_until (wall time, no sim time active): exercises the
    # wrapper -> C++ holder -> clock-type-matched rclcpp::Time target -> clock->sleep_until
    # path directly, synchronously, on the main thread.
    now_ns = clock.now_nanoseconds()
    start = time.monotonic()
    result = sleeper.sleep_until(now_ns + 50_000_000)
    elapsed = time.monotonic() - start
    assert result is True
    assert elapsed >= 0.045, "sleep_until returned too early: %.4fs" % elapsed
    print("NATIVE_CLOCK_SLEEP_UNTIL_WRAPPER_OK")

    # 3. Sim-time sleep_until, driven by a /clock publisher (like the node-clock proof).
    sim_node = session.create_node("native_clock_sleep_sim")
    sim_publisher_node = session.create_node("native_clock_sleep_publisher")
    executor = session.create_executor("single_threaded")
    executor.add_node(sim_node)
    executor.add_node(sim_publisher_node)

    sim_clock = session.create_native_node_clock(sim_node)
    result = sim_node.set_parameter(
        session.rclcpp.Parameter("use_sim_time", True))
    assert result.successful
    deadline = time.monotonic() + 5.0
    while not sim_clock.ros_time_is_active and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert sim_clock.ros_time_is_active

    message_type = load_message_type("rosgraph_msgs", "Clock").cpp_type
    qos = session.rclcpp.QoS(1)
    qos.best_effort()
    publisher = direct_entities.create_managed_publisher(
        sim_publisher_node, message_type, "/clock", qos)
    deadline = time.monotonic() + 5.0
    while (
        publisher.entity().get_subscription_count() < 1
        and time.monotonic() < deadline
    ):
        executor.spin_some()
        time.sleep(0.001)
    assert publisher.entity().get_subscription_count() >= 1

    SIM_DEADLINE_NS = 5_000_000_000  # 5s of sim time

    sim_outcome = cppyy.gbl.clock_sleep_proof.make_outcome()
    cppyy.gbl.clock_sleep_proof.sleep_until_after(
        sim_clock.raw_clock, session.context, SIM_DEADLINE_NS, sim_outcome)
    time.sleep(0.05)
    assert not sim_outcome.done(), (
        "sleep_until resolved before sim time reached the deadline")

    message = message_type()
    current_ns = 0
    step_ns = 250_000_000
    sim_start = time.monotonic()
    sim_wall_deadline = sim_start + 8.0
    while not sim_outcome.done() and time.monotonic() < sim_wall_deadline:
        current_ns += step_ns
        message.clock.sec = current_ns // 1_000_000_000
        message.clock.nanosec = current_ns % 1_000_000_000
        publisher.publish(message)
        executor.spin_some()
        time.sleep(0.01)
    sim_elapsed = time.monotonic() - sim_start
    assert sim_outcome.done(), "sleep_until never resolved as sim time advanced"
    assert sim_outcome.result() is True
    assert sim_elapsed < 8.0, "sim sleep_until took too long: %.3fs" % sim_elapsed
    print("NATIVE_CLOCK_SLEEP_SIM_OK")

    # 4. Context-shutdown interrupt (load-bearing). Runs last: it closes the session.
    interrupt_node = session.create_node("native_clock_sleep_interrupt")
    interrupt_sleeper = session.create_native_clock_sleeper(interrupt_node)
    interrupt_clock = session.create_native_node_clock(interrupt_node)
    assert interrupt_sleeper.clock_address == interrupt_clock.address

    interrupt_outcome = cppyy.gbl.clock_sleep_proof.make_outcome()
    LONG_SLEEP_NS = 10_000_000_000  # 10s wall, never meant to complete normally
    cppyy.gbl.clock_sleep_proof.sleep_for_after(
        interrupt_clock.raw_clock, session.context, LONG_SLEEP_NS, interrupt_outcome)
    time.sleep(0.05)
    assert not interrupt_outcome.done(), (
        "sleep_for resolved before the interrupt was issued")

    interrupt_start = time.monotonic()
    session.close()
    join_deadline = time.monotonic() + 3.0
    while not interrupt_outcome.done() and time.monotonic() < join_deadline:
        time.sleep(0.005)
    interrupt_elapsed = time.monotonic() - interrupt_start
    assert interrupt_outcome.done(), (
        "context shutdown did not interrupt the in-flight sleep_for")
    assert interrupt_outcome.result() is False, (
        "an interrupted sleep_for must return False")
    assert interrupt_elapsed < 3.0, (
        "interrupt took too long: %.3fs" % interrupt_elapsed)
    print("NATIVE_CLOCK_SLEEP_INTERRUPT_OK")
