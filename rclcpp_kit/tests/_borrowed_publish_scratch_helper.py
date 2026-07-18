#!/usr/bin/env python3
"""Isolated reuse, eviction, recovery, and thread-isolation proof."""

from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
import os
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from rclcpp_kit import borrowed_publish


TIMEOUT_S = 10.0
THREADS = 4
MESSAGES_PER_THREAD = 5


def spin_until(executor, condition):
    deadline = time.monotonic() + TIMEOUT_S
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert condition()


def make_graph(name, topic, callback, depth=100):
    context = Context()
    context.init(args=[])
    node = rclpy.create_node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    subscription = node.create_subscription(String, topic, callback, depth)
    publisher = node.create_publisher(String, topic, depth)
    spin_until(executor, lambda: publisher.get_subscription_count() >= 1)
    return context, node, executor, subscription, publisher


def destroy_graph(context, node, executor, subscription, publisher):
    node.destroy_publisher(publisher)
    node.destroy_subscription(subscription)
    executor.remove_node(node)
    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    if context.ok():
        context.shutdown()


def main():
    route = borrowed_publish.prepare(String)
    first_received = []
    first_topic = "/borrowed_publish/scratch_first_%d" % os.getpid()
    first = make_graph(
        "borrowed_publish_scratch_first_%d" % os.getpid(),
        first_topic,
        lambda message: first_received.append(message.data),
    )
    first_context, _, first_executor, _, first_publisher = first

    route.publish(first_publisher, String(data="python-one"))
    cpp_message = route.cpp_message_type()
    cpp_message.data = "cppyy-two!"
    route.publish(first_publisher, cpp_message)
    spin_until(first_executor, lambda: len(first_received) == 2)
    assert first_received == ["python-one", "cppyy-two!"], first_received
    reuse = route._scratch_stats()
    assert reuse["created"] == 1, reuse
    assert reuse["reused"] == 1, reuse
    assert reuse["publishes"] == 2, reuse
    assert reuse["reallocations"] == 1, reuse
    assert 0 < reuse["retained_capacity"] <= 8 * 1024 * 1024, reuse
    print("BORROWED_PUBLISH_SCRATCH_REUSE " + json.dumps(
        reuse, sort_keys=True), flush=True)
    print("BORROWED_PUBLISH_SCRATCH_REUSE_OK", flush=True)

    route._max_retained_serialized_capacity = 64
    route.publish(first_publisher, String(data="x" * 1024))
    eviction = route._scratch_stats()
    assert eviction["created"] == 1, eviction
    assert eviction["evictions"] == 1, eviction
    assert eviction["retained_slots"] == 0, eviction
    assert eviction["peak_capacity"] > 64, eviction
    route._max_retained_serialized_capacity = (
        borrowed_publish._MAX_RETAINED_SERIALIZED_CAPACITY)
    print("BORROWED_PUBLISH_SCRATCH_EVICTION " + json.dumps(
        eviction, sort_keys=True), flush=True)
    print("BORROWED_PUBLISH_SCRATCH_EVICTION_OK", flush=True)

    first_context.shutdown()
    try:
        route.publish(first_publisher, String(data="must-fail"))
    except Exception as exception:
        assert "rcl_publish_serialized_message failed" in str(exception)
    else:
        raise AssertionError("publish on a shut-down context unexpectedly succeeded")
    failed = route._scratch_stats()
    assert failed["created"] == 2, failed
    assert failed["evictions"] == 1, failed
    assert failed["retained_length"] == 0, failed
    destroy_graph(*first)

    second_received = []
    second_topic = "/borrowed_publish/scratch_second_%d" % os.getpid()
    second = make_graph(
        "borrowed_publish_scratch_second_%d" % os.getpid(),
        second_topic,
        lambda message: second_received.append(message.data),
    )
    _, _, second_executor, _, second_publisher = second
    route.publish(second_publisher, String(data="after-failure"))
    spin_until(second_executor, lambda: second_received == ["after-failure"])
    recovered = route._scratch_stats()
    assert recovered["created"] == failed["created"], recovered
    assert recovered["reused"] == failed["reused"] + 1, recovered
    assert recovered["publishes"] == failed["publishes"] + 1, recovered
    print("BORROWED_PUBLISH_SCRATCH_RECOVERY " + json.dumps(
        recovered, sort_keys=True), flush=True)
    print("BORROWED_PUBLISH_SCRATCH_RECOVERY_OK", flush=True)

    main_thread_before = route._scratch_stats()
    barrier = threading.Barrier(THREADS)

    def publish_from_worker(worker):
        barrier.wait(timeout=TIMEOUT_S)
        payloads = [
            "worker-%d-message-%d" % (worker, index)
            for index in range(MESSAGES_PER_THREAD)
        ]
        for payload in payloads:
            route.publish(second_publisher, String(data=payload))
        return payloads, route._scratch_stats()

    with ThreadPoolExecutor(max_workers=THREADS) as pool:
        results = list(pool.map(publish_from_worker, range(THREADS)))

    expected = [payload for payloads, _ in results for payload in payloads]
    spin_until(second_executor, lambda: len(second_received) == 1 + len(expected))
    assert Counter(second_received[1:]) == Counter(expected), second_received
    worker_stats = [stats for _, stats in results]
    assert all(stats["created"] == 1 for stats in worker_stats), worker_stats
    assert all(
        stats["reused"] == MESSAGES_PER_THREAD - 1
        for stats in worker_stats
    ), worker_stats
    assert all(stats["publishes"] == MESSAGES_PER_THREAD for stats in worker_stats), (
        worker_stats)
    assert route._scratch_stats() == main_thread_before
    print("BORROWED_PUBLISH_SCRATCH_THREADS " + json.dumps(
        worker_stats, sort_keys=True), flush=True)
    print("BORROWED_PUBLISH_SCRATCH_THREADS_OK", flush=True)

    destroy_graph(*second)


if __name__ == "__main__":
    main()
