"""Measure elapsed simulation time per wall second from the live ROS clock."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    os.environ.update(ROS_DOMAIN_ID='77', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    rclpy.init()
    node = rclpy.create_node('measure_demo_rtf')
    stamp = [None]
    node.create_subscription(Clock, '/clock', lambda m: stamp.__setitem__(
        0, m.clock.sec + m.clock.nanosec * 1e-9), 10)
    try:
        deadline = time.monotonic() + 10
        while stamp[0] is None:
            if time.monotonic() > deadline:
                raise TimeoutError('No simulation clock received')
            rclpy.spin_once(node, timeout_sec=0.1)
        start_wall, start_sim = time.monotonic(), stamp[0]
        while time.monotonic() - start_wall < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
        wall, sim = time.monotonic() - start_wall, stamp[0] - start_sim
        result = {'wall_seconds': wall, 'sim_seconds': sim, 'rtf': sim / wall}
        text = json.dumps(result, indent=2) + '\n'
        print(text, end='')
        if args.output:
            args.output.write_text(text)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
