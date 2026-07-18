#!/usr/bin/env python3

import time

from rclcpp_kit.native import native
from std_msgs.msg import String


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true")


def main():
    observed = []
    with native(["native-pipeline-test"]) as ros:
        node = ros.create_node("native_pipeline")
        peer = ros.create_node("native_pipeline_peer")
        executor = ros.create_executor()
        executor.add_node(node)
        executor.add_node(peer)

        callback = ros.create_native_callback(
            node,
            String,
            "native_callback_in",
            "set_value(static_cast<int64_t>(message.data.size()));",
        )
        callback_publisher = peer.create_publisher(String, "native_callback_in", 10)

        pipeline = ros.create_fused_pipeline(
            node,
            String,
            String,
            "fused_in",
            "fused_out",
            'output.data = input.data + ":fused";',
        )
        source = peer.create_publisher(String, "fused_in", 10)
        sink = peer.create_subscription(
            String,
            "fused_out",
            lambda message: observed.append(str(message.data)),
            10,
        )
        assert sink is not None

        for _ in range(30):
            executor.spin_some()
        callback_publisher.publish(String(data="1234567"))
        source.publish(String(data="payload"))
        spin_until(
            executor,
            lambda: callback.stats().processed == 1 and bool(observed),
        )

        assert callback.value() == 7
        assert callback.stats().python_boundary_crossings == 0
        assert observed == ["payload:fused"]
        stats = pipeline.stats()
        assert stats.received == 1
        assert stats.processed == 1
        assert stats.published == 1
        assert stats.exceptions == 0
        assert stats.python_boundary_crossings == 0
        assert stats.output_instances == 1
        assert stats.middleware_loaned_messages == 0
        assert stats.allocator_fallbacks == 0
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1

        memory_outputs = {"reuse": [], "loaned": []}
        reuse = ros.create_fused_pipeline(
            node,
            String,
            String,
            "reuse_in",
            "reuse_out",
            'output.data = input.data + ":reuse";',
            output_memory="reuse",
        )
        loaned = ros.create_fused_pipeline(
            node,
            String,
            String,
            "loaned_in",
            "loaned_out",
            'output.data = input.data + ":loaned";',
            output_memory="loaned",
        )
        reuse_source = peer.create_publisher(String, "reuse_in", 10)
        loaned_source = peer.create_publisher(String, "loaned_in", 10)
        reuse_sink = peer.create_subscription(
            String, "reuse_out",
            lambda message: memory_outputs["reuse"].append(str(message.data)), 10)
        loaned_sink = peer.create_subscription(
            String, "loaned_out",
            lambda message: memory_outputs["loaned"].append(str(message.data)), 10)
        assert reuse_sink is not None and loaned_sink is not None
        for _ in range(30):
            executor.spin_some()
        for value in ("first", "second"):
            reuse_source.publish(String(data=value))
            loaned_source.publish(String(data=value))
        spin_until(
            executor,
            lambda: all(len(values) == 2 for values in memory_outputs.values()),
        )
        assert memory_outputs == {
            "reuse": ["first:reuse", "second:reuse"],
            "loaned": ["first:loaned", "second:loaned"],
        }
        reuse_stats = reuse.stats()
        loaned_stats = loaned.stats()
        assert reuse.output_memory == "reuse"
        assert reuse_stats.output_instances == 1
        assert reuse_stats.middleware_loaned_messages == 0
        assert reuse_stats.allocator_fallbacks == 0
        assert loaned.output_memory == "loaned"
        assert loaned_stats.output_instances == 2
        assert (
            loaned_stats.middleware_loaned_messages
            + loaned_stats.allocator_fallbacks
        ) == 2
        assert reuse_stats.compile_cache_hits + reuse_stats.compile_cache_misses == 1
        assert loaned_stats.compile_cache_hits + loaned_stats.compile_cache_misses == 1
        print("FUSED_OUTPUT_MEMORY_OK")

        latest = ros.create_fused_pipeline(
            node,
            String,
            String,
            "latest_in",
            "latest_out",
            "output.data = input.data;",
            delivery="latest",
        )
        batch = ros.create_fused_pipeline(
            node,
            String,
            String,
            "batch_in",
            "batch_out",
            "output.data = input.data;",
            delivery="batch",
            batch_size=4,
            queue_capacity=8,
        )
        latest_source = peer.create_publisher(String, "latest_in", 10)
        batch_source = peer.create_publisher(String, "batch_in", 10)
        latest_sink = peer.create_subscription(
            String, "latest_out", lambda message: None, 10)
        batch_sink = peer.create_subscription(
            String, "batch_out", lambda message: None, 10)
        assert latest_sink is not None and batch_sink is not None
        for _ in range(30):
            executor.spin_some()
        latest_source.publish(String(data="latest"))
        batch_source.publish(String(data="batch"))
        spin_until(
            executor,
            lambda: latest.stats().published == 1 and batch.stats().published == 1,
        )
        assert latest.policy == "latest"
        assert batch.policy == "batch"
        assert latest.stats().python_boundary_crossings == 0
        assert batch.stats().python_boundary_crossings == 0
        print("NATIVE_CALLBACK_OK")
        print("FUSED_EVERY_OK")
        print("FUSED_POLICIES_OK")

    assert all(resource.closed for resource in (
        callback, pipeline, reuse, loaned, latest, batch))
    print("NATIVE_PIPELINE_TEARDOWN_OK")


if __name__ == "__main__":
    main()
