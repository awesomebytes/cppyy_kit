"""Measure real native perception deployments against live sensors or bag replay."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def quantiles(values):
    if not values:
        return {"count": 0, "median": None, "p95": None}
    values = sorted(values)
    return {"count": len(values), "median": values[len(values) // 2],
            "p95": values[min(len(values)-1, int(.95 * len(values)))]}


def measure(mode, args, node, rclpy):
    output_dir = Path(args.output)
    children, logs, reports = [], [], []
    ages = {"objects": [], "image_objects": []}
    counts = {name: [] for name in ages}
    source_arrivals = {name: {} for name in ages}
    wall_latency = {name: [] for name in ages}
    active = False

    def receive(name, message):
        if not active:
            return
        result = json.loads(message.data)
        stamp = result["stamp_sec"] + result["stamp_nanosec"] * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        if age >= 0:
            ages[name].append(age)
        source_arrival = source_arrivals[name].pop(
            (result["stamp_sec"], result["stamp_nanosec"]), None)
        if source_arrival is not None:
            wall_latency[name].append(time.monotonic() - source_arrival)
        counts[name].append(len(result["objects"]))

    from std_msgs.msg import String
    subscriptions = [node.create_subscription(
        String, args.prefix + "/" + name,
        lambda msg, name=name: receive(name, msg), 1) for name in ages]
    from sensor_msgs.msg import Image, PointCloud2
    from rclpy.qos import qos_profile_sensor_data

    def source_receive(name, message):
        entries = source_arrivals[name]
        entries[(message.header.stamp.sec, message.header.stamp.nanosec)] = time.monotonic()
        while len(entries) > 128:
            entries.pop(next(iter(entries)))

    subscriptions.extend(node.create_subscription(
        msg_type, topic, lambda msg, name=name: source_receive(name, msg),
        qos_profile_sensor_data) for name, msg_type, topic in (
            ("objects", PointCloud2, "/head_front_camera/depth/color/points"),
            ("image_objects", Image, "/head_front_camera/color/image_raw")))
    bag = None
    try:
        stages = ("all",) if mode == "composed" else ("filter", "cluster", "image")
        for stage in stages:
            report = output_dir / f"{mode}-{stage}.json"
            log = open(output_dir / f"{mode}-{stage}.log", "w+")
            logs.append(log)
            reports.append(report)
            children.append(subprocess.Popen([
                sys.executable, str(HERE / "pipeline.py"), "--stage", stage,
                "--prefix", args.prefix, "--report", str(report)],
                stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + args.startup_timeout
        while True:
            if any(child.poll() is not None for child in children):
                raise RuntimeError("perception process exited during startup; inspect logs")
            ready = []
            for log in logs:
                log.seek(0)
                ready.append('"event": "ready"' in log.read())
            if all(ready):
                break
            if time.monotonic() > deadline:
                raise TimeoutError("native startup timed out; inspect logs")
            rclpy.spin_once(node, timeout_sec=.1)
        if args.bag:
            bag = subprocess.Popen(["ros2", "bag", "play", args.bag, "--clock"],
                                   stdout=subprocess.DEVNULL)
        # DDS discovery, TF history and first native invocation settle before
        # collecting frame ages. Each pipeline separately reports its CPU interval.
        until = time.monotonic() + args.warmup
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        active = True
        until = time.monotonic() + args.duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        active = False
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        if bag and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            bag.wait(timeout=20)
        for child in children:
            try:
                child.wait(timeout=30)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        for log in logs:
            log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
    deployment = [json.loads(report.read_text()) for report in reports]
    return {"mode": mode, "observer_wall_latency_s": {k: quantiles(v) for k,v in wall_latency.items()},
            "simulated_frame_age_s": {k: quantiles(v) for k,v in ages.items()},
            "object_counts": {k: quantiles(v) for k,v in counts.items()},
            "pipeline_reports": deployment,
            "mean_cpu_cores_sum": sum(r["cpu_cores"] for r in deployment),
            "measurement_s": args.duration, "warmup_s": args.warmup}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("composed", "separate", "both"), default="both")
    p.add_argument("--duration", type=float, default=15)
    p.add_argument("--warmup", type=float, default=3)
    p.add_argument("--startup-timeout", type=float, default=240)
    p.add_argument("--prefix", default="/pick_place_benchmark")
    p.add_argument("--output", default="/tmp/pick-place-benchmark")
    p.add_argument("--bag", help="replay the same ROS bag for each deployment; include TF and camera topics")
    args = p.parse_args()
    Path(args.output).mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    node = rclpy.create_node("pick_place_benchmark", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    try:
        result = {"source": args.bag or "live sensors; sequential runs can see different inputs",
                  "wall_latency_origin": "source callback arrival in external observer; excludes sensor generation and includes observer scheduling",
                  "frame_age_clock": "simulation time; includes transport, queueing and processing",
                  "cpu_interval": "process ready to shutdown request, includes discovery and warmup",
                  "deployments": [measure(mode, args, node, rclpy) for mode in
                      (("composed", "separate") if args.mode == "both" else (args.mode,))]}
        Path(args.output, "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
