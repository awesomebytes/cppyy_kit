#!/usr/bin/env python3
"""Repository demo: native ControllerInterface with real CM and mock hardware.

Run: pixi run -e control demo-control-native
Expected: two joints converge to their target within 0.002 rad.
Python loads the cached controller, sets goals, and drives the manager. The
controller's lifecycle and update virtual methods execute compiled C++ only.
"""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "49")

import control_kit as ck  # noqa: E402
from control_kit.native_control import (  # noqa: E402
    compile_native_controller,
    make_native_position_controller,
)
from rclcpp_kit.bringup_rclcpp import bringup_rclcpp  # noqa: E402


def main():
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    joints = ["shoulder", "elbow"]
    controller = make_native_position_controller()
    rig = ck.make_controller_manager(ck.mock_system_urdf(joints), update_rate=100)
    try:
        rig.add_native_controller(
            controller,
            "native_servo",
            parameters={
                "joints": joints,
                "gain": 6.0,
                "speed_limits": [0.5, 0.4],
                "lower_limits": [-1.0, -1.0],
                "upper_limits": [1.0, 1.0],
            },
        )
        if not rig.configure("native_servo") or not rig.activate(["native_servo"]):
            raise RuntimeError("native servo failed to configure or activate")
        goal = [0.6, -0.35]
        if not controller.set_target(goal):
            raise RuntimeError("goal rejected")
        if controller.set_target([float("nan"), 0.0]):
            raise RuntimeError("nonfinite goal accepted")
        rig.run(seconds=3.0)
        rig.update()  # read the final hardware command back through GenericSystem
        positions = [ck.read_state(controller, i) for i in range(len(joints))]
        error = max(abs(q - g) for q, g in zip(positions, goal))
        print("Native controller library:", compile_native_controller()["so"])
        print("Goal:", goal, "measured:", positions)
        print("Maximum error: %.6f rad" % error)
        if error > 0.002:
            raise RuntimeError("native servo failed to converge")
        print("OK: native ControllerInterface updated through real controller_manager")
    finally:
        rig._teardown()


if __name__ == "__main__":
    main()
