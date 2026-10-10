"""Fill in a Python controller update; run against ros2_control mock hardware."""
import argparse
import json
import math
import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "79")
import numpy as np
import control_kit as ck
from rclcpp_kit.bringup_rclcpp import bringup_rclcpp

JOINTS = ["shoulder", "elbow"]


class TrackingController(ck.ControllerInterface):
    def __init__(self):
        super().__init__()
        self.t = 0.0
        self.samples = []

    def on_init(self):
        return ck.CallbackReturn.SUCCESS

    def command_interface_configuration(self):
        return ck.interface_config([f"{j}/position" for j in JOINTS])

    def state_interface_configuration(self):
        return ck.interface_config([f"{j}/position" for j in JOINTS])

    def on_configure(self, state):
        return ck.CallbackReturn.SUCCESS

    def on_activate(self, state):
        self.t = 0.0
        self.samples.clear()
        return ck.CallbackReturn.SUCCESS

    def on_deactivate(self, state):
        return ck.CallbackReturn.SUCCESS

    def update(self, stamp, period):
        """Advance t by period.seconds(), then track [0.3*sin(t),0.2*cos(t)].

        For each position interface, command p + dt*20*(reference-p).
        Record (t, references, measured positions) in samples. Return OK.
        This law targets GenericSystem position mirroring, not robot dynamics.
        """
        dt = period.seconds()
        self.t += dt
        references = [0.3 * math.sin(self.t), 0.2 * math.cos(self.t)]
        positions = []
        for i in range(ck.n_state_interfaces(self)):
            position = ck.read_state(self, i)
            command = position + dt * 20.0 * (references[i] - position)
            ck.write_command(self, i, command)
            positions.append(position)
        self.samples.append((self.t, references, positions))
        return ck.return_type.OK


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=int, default=1000)
    parser.add_argument("--seconds", type=float, default=3)
    args = parser.parse_args()
    if args.rate < 100 or not math.isfinite(args.seconds) or args.seconds < 1:
        parser.error("rate >= 100 Hz and seconds >= 1 required")
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    ck.bringup_control()
    rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS), update_rate=args.rate)
    controller = TrackingController()
    rig.add_python_controller(controller, "tracking")
    assert rig.configure("tracking")
    assert rig.activate(["tracking"])
    stamps = []
    started = time.perf_counter()
    cycles = rig.run(args.seconds, args.rate, on_cycle=lambda _: stamps.append(time.perf_counter()))
    elapsed = time.perf_counter() - started
    intervals = np.diff(stamps)
    tail = controller.samples[len(controller.samples)//2:]
    error = max(abs(r-p) for _, refs, pos in tail for r, p in zip(refs, pos))
    result = {"cycles": cycles, "target_hz": args.rate, "elapsed_s": elapsed,
              "achieved_hz": cycles / elapsed,
              "interval_p99_ms": float(np.percentile(intervals, 99)*1000),
              "late_intervals": int(np.count_nonzero(intervals > 1.5/args.rate)),
              "tail_max_error_rad": error, "hardware": "mock_components/GenericSystem"}
    rig._teardown()
    print("RESULT " + json.dumps(result))


if __name__ == "__main__":
    main()
