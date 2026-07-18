#!/usr/bin/env python3

import time

from rclcpp_kit.native import native
from rclcpp_kit.native_pipeline import (
    create_fused_pipeline,
    create_native_callback,
)
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

        callback = create_native_callback(
            ros,
            node,
            String,
            "native_callback_in",
            "set_value(static_cast<int64_t>(message.data.size()));",
        )
        callback_publisher = peer.create_publisher(String, "native_callback_in", 10)

        pipeline = create_fused_pipeline(
            ros,
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

        latest = create_fused_pipeline(
            ros,
            node,
            String,
            String,
            "latest_in",
            "latest_out",
            "output.data = input.data;",
            delivery="latest",
        )
        batch = create_fused_pipeline(
            ros,
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

    assert callback.closed and pipeline.closed and latest.closed and batch.closed
    print("NATIVE_PIPELINE_TEARDOWN_OK")


if __name__ == "__main__":
    main()
