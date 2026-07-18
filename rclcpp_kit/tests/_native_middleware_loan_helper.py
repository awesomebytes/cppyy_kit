#!/usr/bin/env python3

import time

from rclpy.utilities import get_rmw_implementation_identifier
from rclcpp_kit.native import native, publisher_capabilities
from std_msgs.msg import UInt64


EXPECTED_RMW = "rmw_fastrtps_cpp"
MESSAGE_COUNT = 8


def spin_until(executor, predicate, description, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for %s" % description)


def main():
    received = []
    session = native(["native-middleware-loan-test"])

    with session as ros:
        assert get_rmw_implementation_identifier() == EXPECTED_RMW

        worker = ros.create_node("native_middleware_loan_worker")
        peer = ros.create_node("native_middleware_loan_peer")
        executor = ros.create_executor()
        executor.add_node(worker)
        executor.add_node(peer)

        capability_publisher = worker.create_publisher(
            UInt64, "native_middleware_loan_capability", 16)
        capability = publisher_capabilities(capability_publisher)
        assert capability["loaned_messages"] is True

        pipeline = ros.create_fused_pipeline(
            worker,
            UInt64,
            UInt64,
            "native_middleware_loan_input",
            "native_middleware_loan_output",
            "output.data = input.data * 2;",
            qos_depth=16,
            output_memory="loaned",
        )
        source = peer.create_publisher(
            UInt64, "native_middleware_loan_input", 16)
        sink = peer.create_subscription(
            UInt64,
            "native_middleware_loan_output",
            lambda message: received.append(int(message.data)),
            16,
        )
        assert sink is not None

        spin_until(
            executor,
            lambda: (
                worker.count_subscribers("native_middleware_loan_input") >= 1
                and peer.count_publishers("native_middleware_loan_output") >= 1
            ),
            "pipeline discovery",
        )
        for value in range(1, MESSAGE_COUNT + 1):
            source.publish(UInt64(data=value))
        spin_until(
            executor,
            lambda: len(received) == MESSAGE_COUNT,
            "all loaned outputs",
        )

        assert received == [value * 2 for value in range(1, MESSAGE_COUNT + 1)]
        stats = pipeline.stats()
        assert stats.received == MESSAGE_COUNT
        assert stats.processed == MESSAGE_COUNT
        assert stats.published == MESSAGE_COUNT
        assert stats.output_instances == MESSAGE_COUNT
        assert stats.middleware_loaned_messages == MESSAGE_COUNT
        assert stats.allocator_fallbacks == 0
        assert stats.exceptions == 0
        assert stats.python_boundary_crossings == 0
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1

        print("MIDDLEWARE_LOAN_RMW=%s" % EXPECTED_RMW)
        print("MIDDLEWARE_LOAN_COUNT=%d" % stats.middleware_loaned_messages)

    assert session.closed
    assert pipeline.closed
    print("MIDDLEWARE_LOAN_PROOF_OK")


if __name__ == "__main__":
    main()
