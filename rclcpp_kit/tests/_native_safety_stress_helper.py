#!/usr/bin/env python3

import argparse
import json
import os
from pathlib import Path
import time

from rclcpp_kit.native import native
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


TIMEOUT_S = 20.0
RETAINED_COUNT = 48
OVERFLOW_COUNT = 200
MEMORY_COUNT = 8

RETAINED_TRANSFORM = """
std::this_thread::sleep_for(std::chrono::milliseconds(2));
output.data = input.data;
"""
OVERFLOW_TRANSFORM = """
std::this_thread::sleep_for(std::chrono::milliseconds(200));
output.data = input.data;
"""
EXCEPTION_TRANSFORM = """
if (input.data == "explode") {
  throw std::runtime_error("expected transform failure");
}
output.data = input.data;
"""
MEMORY_TRANSFORM = """
output.data = input.data + ":native";
"""
CONCURRENT_CALLBACK = """
static std::atomic<int> active{0};
static std::atomic<int> maximum{0};
const int current = active.fetch_add(1, std::memory_order_acq_rel) + 1;
int observed = maximum.load(std::memory_order_acquire);
while (current > observed &&
       !maximum.compare_exchange_weak(observed, current,
                                      std::memory_order_acq_rel)) {}
std::this_thread::sleep_for(std::chrono::milliseconds(100));
set_value(maximum.load(std::memory_order_acquire));
active.fetch_sub(1, std::memory_order_acq_rel);
"""
EXCEPTION_SERVICE = """
if (!request->data) {
  throw std::runtime_error("expected service failure");
}
response->success = true;
response->message = "native-success";
"""


def wait_until(executor, predicate, description, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    if not predicate():
        raise AssertionError("timed out waiting for %s" % description)


def loaded_native_glue():
    """Return Linux mappings proving cached native glue is loaded this run."""
    try:
        mappings = Path("/proc/self/maps").read_text().splitlines()
    except OSError:
        return []
    paths = set()
    for line in mappings:
        path = line.rsplit(None, 1)[-1]
        if (
            path.endswith(".so")
            and (
                "/native-pipelines/" in path
                or "/native-services/" in path
            )
        ):
            paths.add(path)
    return sorted(paths)


def create_resources(ros, prefix):
    pipeline_node = ros.create_node("native_safety_pipeline")
    callback_node_a = ros.create_node("native_safety_callback_a")
    callback_node_b = ros.create_node("native_safety_callback_b")
    service_node = ros.create_node("native_safety_service")
    resources = {
        "retained": ros.create_fused_pipeline(
            pipeline_node,
            String,
            String,
            prefix + "/retained_in",
            prefix + "/retained_out",
            RETAINED_TRANSFORM,
            delivery="batch",
            qos_depth=256,
            batch_size=8,
            queue_capacity=128,
            includes=("chrono", "thread"),
        ),
        "overflow": ros.create_fused_pipeline(
            pipeline_node,
            String,
            String,
            prefix + "/overflow_in",
            prefix + "/overflow_out",
            OVERFLOW_TRANSFORM,
            delivery="batch",
            qos_depth=512,
            batch_size=2,
            queue_capacity=2,
            includes=("chrono", "thread"),
        ),
        "transform_exception": ros.create_fused_pipeline(
            pipeline_node,
            String,
            String,
            prefix + "/exception_in",
            prefix + "/exception_out",
            EXCEPTION_TRANSFORM,
            qos_depth=32,
            includes=("stdexcept",),
        ),
        "reuse": ros.create_fused_pipeline(
            pipeline_node,
            String,
            String,
            prefix + "/reuse_in",
            prefix + "/reuse_out",
            MEMORY_TRANSFORM,
            qos_depth=32,
            output_memory="reuse",
        ),
        "loaned": ros.create_fused_pipeline(
            pipeline_node,
            String,
            String,
            prefix + "/loaned_in",
            prefix + "/loaned_out",
            MEMORY_TRANSFORM,
            qos_depth=32,
            output_memory="loaned",
        ),
        "callback_a": ros.create_native_callback(
            callback_node_a,
            String,
            prefix + "/callback_a",
            CONCURRENT_CALLBACK,
            qos_depth=32,
            includes=("atomic", "chrono", "thread"),
        ),
        "callback_b": ros.create_native_callback(
            callback_node_b,
            String,
            prefix + "/callback_b",
            CONCURRENT_CALLBACK,
            qos_depth=32,
            includes=("atomic", "chrono", "thread"),
        ),
        "service": ros.create_native_service(
            service_node,
            SetBool,
            prefix + "/service",
            EXCEPTION_SERVICE,
            includes=("stdexcept",),
        ),
    }
    return (
        (pipeline_node, callback_node_a, callback_node_b, service_node),
        resources,
    )


def compile_only():
    prefix = "/native_safety_prebuild_%d" % os.getpid()
    with native(["native-safety-prebuild"]) as ros:
        _, resources = create_resources(ros, prefix)
    assert all(resource.closed for resource in resources.values())
    print("NATIVE_SAFETY_PREBUILD_OK")


def run_stress(evidence_path):
    prefix = "/native_safety_%d" % os.getpid()
    context = Context()
    context.init(args=[])
    peer = Node("native_safety_peer", context=context)
    peer_executor = SingleThreadedExecutor(context=context)
    peer_executor.add_node(peer)

    session = native(["native-safety-stress"])
    session.open()
    nodes, resources = create_resources(session, prefix)
    native_executor = session.create_executor("multi_threaded", threads=4)
    for node in nodes:
        native_executor.add_node(node)

    retained_replies = []
    memory_replies = {"reuse": [], "loaned": []}
    retained_sub = peer.create_subscription(
        String, prefix + "/retained_out",
        lambda message: retained_replies.append(str(message.data)), 256)
    retained_pub = peer.create_publisher(String, prefix + "/retained_in", 256)
    reuse_sub = peer.create_subscription(
        String, prefix + "/reuse_out",
        lambda message: memory_replies["reuse"].append(str(message.data)), 32)
    loaned_sub = peer.create_subscription(
        String, prefix + "/loaned_out",
        lambda message: memory_replies["loaned"].append(str(message.data)), 32)
    reuse_pub = peer.create_publisher(String, prefix + "/reuse_in", 32)
    loaned_pub = peer.create_publisher(String, prefix + "/loaned_in", 32)
    overflow_pub = peer.create_publisher(String, prefix + "/overflow_in", 512)
    exception_pub = peer.create_publisher(String, prefix + "/exception_in", 32)
    callback_pub_a = peer.create_publisher(String, prefix + "/callback_a", 32)
    callback_pub_b = peer.create_publisher(String, prefix + "/callback_b", 32)
    service_client = peer.create_client(SetBool, prefix + "/service")

    native_thread = session.start_executor(native_executor)

    evidence = {
        "schema": "rclcpp_kit.native-safety/v1",
        "executor": {"kind": "multi_threaded", "threads": 4},
        "loaded_native_glue": loaded_native_glue(),
        "retained_input": {},
        "bounded_overflow": {},
        "exceptions": {},
        "output_memory": {},
        "shutdown": {},
        "python_boundary_crossings": {},
    }
    try:
        publishers = (
            retained_pub,
            reuse_pub,
            loaned_pub,
            overflow_pub,
            exception_pub,
            callback_pub_a,
            callback_pub_b,
        )
        wait_until(
            peer_executor,
            lambda: all(pub.get_subscription_count() >= 1 for pub in publishers),
            "native subscription discovery",
        )
        wait_until(
            peer_executor,
            lambda: all(
                peer.count_publishers(prefix + "/" + name + "_out") >= 1
                for name in ("retained", "reuse", "loaned")
            ),
            "native output publishers discovery",
        )
        wait_until(
            peer_executor,
            service_client.service_is_ready,
            "native service discovery",
        )

        reused_message = String()
        retained_expected = []
        for index in range(RETAINED_COUNT):
            value = "retained-%03d" % index
            reused_message.data = value
            retained_expected.append(value)
            retained_pub.publish(reused_message)

        memory_expected = ["memory-%02d:native" % index
                           for index in range(MEMORY_COUNT)]
        for index in range(MEMORY_COUNT):
            message = String(data="memory-%02d" % index)
            reuse_pub.publish(message)
            loaned_pub.publish(message)

        exception_pub.publish(String(data="before"))
        exception_pub.publish(String(data="explode"))
        exception_pub.publish(String(data="after"))

        callback_pub_a.publish(String(data="a"))
        callback_pub_b.publish(String(data="b"))

        for index in range(OVERFLOW_COUNT):
            overflow_pub.publish(String(data="overflow-%03d" % index))

        def overflow_pending():
            stats = resources["overflow"].stats()
            pending = stats.received - stats.processed - stats.dropped
            return stats.dropped > 0 and pending > 0

        wait_until(
            peer_executor,
            overflow_pending,
            "bounded overflow with pending work",
        )
        overflow_before = resources["overflow"].stats()
        pending = (
            overflow_before.received
            - overflow_before.processed
            - overflow_before.dropped
        )
        close_started = time.monotonic()
        resources["overflow"].close()
        close_ms = (time.monotonic() - close_started) * 1000.0
        overflow_after = resources["overflow"].stats()
        assert overflow_before.dropped > 0
        assert pending > 0
        assert overflow_after.received == (
            overflow_after.processed + overflow_after.dropped)
        assert overflow_after.processed == overflow_after.published
        assert close_ms < 5000.0
        evidence["bounded_overflow"] = overflow_after.to_dict()
        evidence["bounded_overflow"]["submitted"] = OVERFLOW_COUNT
        evidence["shutdown"] = {
            "pending_before_close": pending,
            "close_ms": round(close_ms, 3),
            "drained": True,
        }

        wait_until(
            peer_executor,
            lambda: len(retained_replies) == RETAINED_COUNT,
            "retained pipeline outputs",
        )
        wait_until(
            peer_executor,
            lambda: resources["transform_exception"].stats().received == 3,
            "transform exception accounting",
        )
        wait_until(
            peer_executor,
            lambda: resources["callback_a"].stats().processed == 1
            and resources["callback_b"].stats().processed == 1,
            "concurrent callbacks",
        )
        assert retained_replies == retained_expected
        retained_stats = resources["retained"].stats()
        assert retained_stats.received == RETAINED_COUNT
        assert retained_stats.processed == RETAINED_COUNT
        assert retained_stats.published == RETAINED_COUNT
        assert retained_stats.dropped == 0
        assert retained_stats.exceptions == 0
        evidence["retained_input"] = retained_stats.to_dict()

        wait_until(
            peer_executor,
            lambda: all(len(values) == MEMORY_COUNT
                        for values in memory_replies.values()),
            "explicit output-memory policy streams",
        )
        assert memory_replies == {
            "reuse": memory_expected,
            "loaned": memory_expected,
        }
        reuse_stats = resources["reuse"].stats()
        loaned_stats = resources["loaned"].stats()
        assert reuse_stats.received == MEMORY_COUNT
        assert reuse_stats.processed == MEMORY_COUNT
        assert reuse_stats.published == MEMORY_COUNT
        assert reuse_stats.output_instances == 1
        assert reuse_stats.middleware_loaned_messages == 0
        assert reuse_stats.allocator_fallbacks == 0
        assert loaned_stats.received == MEMORY_COUNT
        assert loaned_stats.processed == MEMORY_COUNT
        assert loaned_stats.published == MEMORY_COUNT
        assert loaned_stats.output_instances == MEMORY_COUNT
        assert (
            loaned_stats.middleware_loaned_messages
            + loaned_stats.allocator_fallbacks
        ) == MEMORY_COUNT
        for stats in (reuse_stats, loaned_stats):
            assert stats.compile_cache_hits + stats.compile_cache_misses == 1
        evidence["output_memory"] = {
            "reuse": reuse_stats.to_dict(),
            "loaned": loaned_stats.to_dict(),
            "ordered_values": memory_expected,
        }

        transform_stats = resources["transform_exception"].stats()
        assert transform_stats.received == 3
        assert transform_stats.processed == 2
        assert transform_stats.published == 2
        assert transform_stats.exceptions == 1

        concurrency = min(
            resources["callback_a"].value(),
            resources["callback_b"].value(),
        )
        assert concurrency >= 2
        evidence["executor"]["observed_callback_concurrency"] = concurrency

        def call_service(value):
            future = service_client.call_async(SetBool.Request(data=value))
            wait_until(peer_executor, future.done, "native service response")
            return future.result()

        success = call_service(True)
        failure = call_service(False)
        assert success.success is True
        assert success.message == "native-success"
        assert failure.success is False
        service_stats = resources["service"].stats()
        assert service_stats.requests == 1
        assert service_stats.exceptions == 1
        evidence["exceptions"] = {
            "transform": transform_stats.to_dict(),
            "service": service_stats.to_dict(),
        }

        session_close_started = time.monotonic()
        session.close()
        session_close_ms = (time.monotonic() - session_close_started) * 1000.0
        assert native_thread.closed
        assert not native_thread.running
        assert native_thread.exceptions == 0
        assert all(resource.closed for resource in resources.values())
        assert session_close_ms < 5000.0
        evidence["shutdown"]["session_close_ms"] = round(
            session_close_ms, 3)
        evidence["python_boundary_crossings"] = {
            name: resource.stats().python_boundary_crossings
            for name, resource in resources.items()
        }
        assert not any(evidence["python_boundary_crossings"].values())
    finally:
        if not session.closed:
            session.close()
        peer.destroy_client(service_client)
        peer.destroy_publisher(callback_pub_b)
        peer.destroy_publisher(callback_pub_a)
        peer.destroy_publisher(exception_pub)
        peer.destroy_publisher(overflow_pub)
        peer.destroy_publisher(retained_pub)
        peer.destroy_publisher(loaned_pub)
        peer.destroy_publisher(reuse_pub)
        peer.destroy_subscription(loaned_sub)
        peer.destroy_subscription(reuse_sub)
        peer.destroy_subscription(retained_sub)
        peer_executor.remove_node(peer)
        peer_executor.shutdown(timeout_sec=1.0)
        peer.destroy_node()
        context.shutdown()

    if evidence_path is not None:
        evidence_path.parent.mkdir(parents=True, exist_ok=True)
        evidence_path.write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print("NATIVE_RETAINED_INPUT_OK")
    print("NATIVE_BOUNDED_OVERFLOW_OK")
    print("NATIVE_EXCEPTION_CONTAINMENT_OK")
    print("NATIVE_MULTITHREADED_OK")
    print("NATIVE_PENDING_SHUTDOWN_OK")
    print("NATIVE_OUTPUT_MEMORY_OK")
    print("NATIVE_SAFETY_STRESS_OK")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--compile-only", action="store_true")
    evidence = os.environ.get("CPPYY_KIT_NATIVE_SAFETY_EVIDENCE")
    parser.add_argument(
        "--evidence",
        type=Path,
        default=Path(evidence) if evidence else None,
    )
    args = parser.parse_args()
    if args.compile_only:
        compile_only()
    else:
        run_stress(args.evidence)


if __name__ == "__main__":
    main()
