"""Single-process native ROS stages with a Python main-thread supervisor."""
import hashlib
import json
from pathlib import Path
import sys
import time

import cppyy
from rclcpp_kit.native import native
from std_msgs.msg import Float64MultiArray

HEADER = Path(__file__).with_name("kernels.hpp")


def frame(sequence):
    return [float(sequence)] + [float(sequence % 17)] * 256


def expected(sequence):
    """Independent Python oracle, encoded into the native sink's integer slot."""
    return sequence * 1000000 + sum(2 * x + 1 for x in frame(sequence)[1:])


def wait_for(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError("native composition did not reach expected state")
        time.sleep(0.002)


def run():
    # Include the content identity in generated source, since the pipeline cache
    # keys include header paths rather than transitive header contents.
    identity = hashlib.sha256(HEADER.read_bytes()).hexdigest()
    cppyy.include(str(HEADER))
    kernels = cppyy.gbl.quick_composition
    for sequence in (1, 8, 19):
        output = kernels.summary(kernels.affine(frame(sequence)))
        assert int(output[0]) * 1000000 + int(output[1]) == expected(sequence)

    with native(["quick-composition"]) as ros:
        nodes = [ros.create_node(name, use_intra_process=True) for name in (
            "quick_source", "quick_affine", "quick_summary", "quick_sink")]
        executor = ros.create_executor()
        for node in nodes:
            executor.add_node(node)
        affine = ros.create_fused_pipeline(
            nodes[1], Float64MultiArray, Float64MultiArray,
            "quick/raw", "quick/affine",
            "// " + identity + "\nquick_composition::simulate_work();\n"
            "output.data = quick_composition::affine(input.data);",
            includes=(str(HEADER),), delivery="latest", qos_depth=256)
        summary = ros.create_fused_pipeline(
            nodes[2], Float64MultiArray, Float64MultiArray,
            "quick/affine", "quick/summary",
            "// " + identity + "\n"
            "output.data = quick_composition::summary(input.data);",
            includes=(str(HEADER),), qos_depth=256)
        sink = ros.create_native_callback(
            nodes[3], Float64MultiArray, "quick/summary",
            "set_value(static_cast<int64_t>(message.data.at(0)) * 1000000 + "
            "static_cast<int64_t>(message.data.at(1)));", qos_depth=256)
        source = nodes[0].create_publisher(Float64MultiArray, "quick/raw", 256)
        thread = ros.start_executor(executor)
        wait_for(lambda: source.get_subscription_count() == 1)
        wait_for(lambda: nodes[1].count_subscribers("quick/affine") == 1)
        wait_for(lambda: nodes[2].count_subscribers("quick/summary") == 1)

        # Sequential inputs test full ROS output parity without intentional loss.
        for sequence in (1, 8, 19):
            source.publish(Float64MultiArray(data=frame(sequence)))
            wait_for(lambda: sink.value() == expected(sequence))
        # A slow first stage and a fast producer exercise its one-pending-frame
        # policy. DDS/intra-process history has its own bounded depth of 256.
        for sequence in range(100, 164):
            source.publish(Float64MultiArray(data=frame(sequence)))

        # Hold the GIL in the main thread. Native execution needs no Python
        # callback. This checks progress, not a maximum UI latency guarantee.
        old_interval = sys.getswitchinterval()
        try:
            sys.setswitchinterval(1.0)
            before = sink.stats().processed
            deadline = time.monotonic() + 0.3
            heartbeats = 0
            while time.monotonic() < deadline:
                heartbeats += 1
            progressed_while_python_busy = sink.stats().processed > before
        finally:
            sys.setswitchinterval(old_interval)
        wait_for(lambda: sink.value() == expected(163))
        wait_for(lambda: affine.stats().received == 67)
        wait_for(lambda: affine.stats().processed + affine.stats().coalesced == 67)
        wait_for(lambda: sink.stats().processed == affine.stats().processed)
        stats = {name: resource.stats().to_dict() for name, resource in (
            ("affine", affine), ("summary", summary), ("sink", sink))}
        assert progressed_while_python_busy
        assert heartbeats > 0
        assert stats["affine"]["coalesced"] > 0
        assert all(s["exceptions"] == 0 for s in stats.values())
        assert all(s["python_boundary_crossings"] == 0 for s in stats.values())
        assert thread.running
    assert thread.closed and not thread.running and thread.exceptions == 0
    assert all(resource.closed for resource in (affine, summary, sink))
    # Session close is idempotent and the final result is readable after join.
    ros.close()
    return {"direct_parity_cases": 3, "ros_parity_cases": 3,
            "busy_main_thread_iterations": heartbeats,
            "native_progress_while_python_busy": progressed_while_python_busy,
            "latest_sequence": 163, "joined_shutdown": thread.closed,
            "stages": stats}


if __name__ == "__main__":
    print(json.dumps(run(), indent=2))
