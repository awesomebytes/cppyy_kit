"""Run unchanged native perception stages together or in separate ROS processes."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import resource
import signal
import threading
import time

HERE = Path(__file__).resolve().parent


def build_pipeline(ros, args):
    # Keep imports inside the measured startup interval, including native loading.
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import String
    import perception

    selected = ("filter", "cluster", "image") if args.stage == "all" else (args.stage,)
    options = perception.ros_pipeline_options()
    identity = hashlib.sha256(b"".join(
        p.read_bytes() for p in sorted((HERE / "native").glob("*.hpp")))).hexdigest()
    prefix = args.prefix.rstrip("/")
    # Each stage has the same code, topic, and queue policy in both deployments.
    specs = {
        "filter": (PointCloud2, PointCloud2, args.cloud_topic,
                   prefix + "/filtered_points",
                   "sensor_msgs::msg::PointCloud2 transformed; "
                   "publish = pick_place::transform_frame(input, transformed, "
                   + json.dumps(args.fixed_frame) + "); "
                   "if (publish) pick_place::cloud_filter(transformed, output);"),
        "cluster": (PointCloud2, String, prefix + "/filtered_points",
                    prefix + "/objects",
                    "pick_place::CloudConfig config; config.leaf=0; "
                    "config.lower={" + ",".join(map(str,args.roi[:3])) + "}; "
                    "config.upper={" + ",".join(map(str,args.roi[3:])) + "}; "
                    "output.data = pick_place::cloud_json(input, config);"),
        "image": (Image, String, args.image_topic,
                  prefix + "/image_objects",
                  "output.data = pick_place::image_json(input);"),
    }
    stages = {}
    nodes = []
    executor = ros.create_executor()
    if "filter" in selected:
        import cppyy
        import cppyy_kit
        from rclcpp_kit import tf
        from rclcpp_kit.bringup_rclcpp import ros2_include_paths
        tf_node = ros.create_node("pick_place_tf")
        nodes.append(tf_node)
        listener = ros.register_resource(tf.TransformListener(tf_node, spin_thread=False))
        executor.add_node(tf_node)
        tf_header = HERE / "native" / "frame_transform.hpp"
        tf_source = (HERE / "native" / "frame_transform.cpp").read_text()
        tf_source = tf_source.replace('#include "frame_transform.hpp"', tf_header.read_text())
        from cppyy_kit.cache import prebuild
        prebuild(
            tf_source, decls=tf_header.read_text(), name="pick_place_frame_transform",
            include_paths=tuple(ros2_include_paths()) + tuple(options["include_paths"]),
            library_paths=options["library_paths"], libraries=("tf2",))
        cppyy_kit.cppdef_cached(
            tf_source, decls=tf_header.read_text(), name="pick_place_frame_transform",
            include_paths=tuple(ros2_include_paths()) + tuple(options["include_paths"]),
            library_paths=options["library_paths"], libraries=("tf2",))
        cppyy.gbl.pick_place.set_frame_buffer(listener.buffer)
    for name in selected:
        in_type, out_type, in_topic, out_topic, body = specs[name]
        node = ros.create_node("pick_place_" + name, use_intra_process=True)
        nodes.append(node)
        executor.add_node(node)
        stages[name] = ros.create_fused_pipeline(
            node, in_type, out_type, in_topic, out_topic,
            "// " + identity + "\n" + body,
            includes=tuple(str(HERE / "native" / h) for h in (
                ("cloud_ros.hpp", "frame_transform.hpp") if name == "filter"
                else ("cloud_ros.hpp",) if name == "cluster"
                else ("image_ros.hpp",))), delivery="latest", qos_depth=1,
            reliability="best_effort" if name != "cluster" else "reliable",
            **options)
    thread = ros.start_executor(executor)
    return executor, thread, stages, nodes


class PerceptionPipeline:
    """Own native perception workers for a Python task controller in this process."""

    def __init__(self, stage="all", **overrides):
        self.args = parser().parse_args([])
        self.args.stage = stage
        for name, value in overrides.items():
            if not hasattr(self.args, name):
                raise TypeError("unknown pipeline option: " + name)
            setattr(self.args, name, value)
        self.ros = None

    def start(self):
        from rclcpp_kit.native import native
        self.ros = native(["pick_place_" + self.args.stage])
        try:
            self.executor, self.thread, self.stages, self.nodes = build_pipeline(self.ros, self.args)
        except BaseException:
            self.ros.close()
            raise
        return self

    def close(self):
        if self.ros is not None:
            self.ros.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()


def run(args):
    started = time.monotonic()
    stop = threading.Event()
    previous = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, lambda *_: stop.set())
    try:
        options = vars(args).copy()
        options.pop("stage")
        with PerceptionPipeline(stage=args.stage, **options) as pipeline:
            stages, thread = pipeline.stages, pipeline.thread
            ready = time.monotonic()
            cpu_started = time.process_time()
            print(json.dumps({"event": "ready", "stage": args.stage,
                              "startup_s": ready - started,
                              "prefix": args.prefix}), flush=True)
            deadline = ready + args.duration if args.duration else float("inf")
            while not stop.wait(min(0.2, max(0.0, deadline - time.monotonic()))):
                if time.monotonic() >= deadline:
                    break
                if not thread.running:
                    raise RuntimeError("native executor stopped before shutdown")
            # Include only ready-to-stop work in the CPU interval. Compilation is
            # startup, and teardown joins workers before reading final counters.
            warm_wall = time.monotonic() - ready
            warm_cpu = time.process_time() - cpu_started
        report = {"stage": args.stage, "startup_s": ready - started,
                  "warm_wall_s": warm_wall, "warm_cpu_s": warm_cpu,
                  "cpu_cores": warm_cpu / warm_wall if warm_wall else None,
                  "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                  "joined_shutdown": thread.closed and not thread.running,
                  "executor_exceptions": thread.exceptions,
                  "stages": {name: obj.stats().to_dict() for name, obj in stages.items()}}
        if args.report:
            Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({"event": "stopped", **report}), flush=True)
        return report
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", choices=("all", "filter", "cluster", "image"), default="all")
    p.add_argument("--cloud-topic", default="/head_front_camera/depth/color/points")
    p.add_argument("--image-topic", default="/head_front_camera/color/image_raw")
    p.add_argument("--prefix", default="/pick_place")
    p.add_argument("--fixed-frame", default="odom")
    p.add_argument("--roi", type=float, nargs=6,
                   default=(-3.1, 1.9, .95, -2.2, 2.6, 1.4),
                   metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"))
    p.add_argument("--duration", type=float, default=0, help="ready-state seconds; zero runs until interrupted")
    p.add_argument("--report", help="write final startup, CPU and bounded-queue counters as JSON")
    return p


if __name__ == "__main__":
    run(parser().parse_args())
