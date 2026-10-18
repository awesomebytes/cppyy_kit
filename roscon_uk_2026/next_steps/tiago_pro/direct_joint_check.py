"""Check direct native head/arm trajectories and actual joint tracking."""
import argparse
import json
from pathlib import Path
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).parent / "moveit"))
from check import Check


def positions(check, names):
    values = dict(zip(check.latest.name, check.latest.position))
    return [values[name] for name in names]


def audit(check, names, initial, target):
    state = check.state()
    trajectory = JointTrajectory(joint_names=names)
    for step in range(11):
        values = [a + (b - a) * step / 10 for a, b in zip(initial, target)]
        for name, value in zip(names, values):
            state.joint_state.position[state.joint_state.name.index(name)] = value
        if not check.within_bounds(state) or not check.validity(state).valid:
            raise RuntimeError("Direct trajectory sample violates limits or collision scene")
        trajectory.points.append(JointTrajectoryPoint(positions=values))
    return check.audit(trajectory)


def command(check, controller, names, target):
    before = positions(check, names)
    safety = audit(check, names, before, target)
    client = ActionClient(check.node, FollowJointTrajectory,
                          f"/{controller}/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=15):
        raise RuntimeError(f"Unavailable controller: {controller}")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    point = JointTrajectoryPoint(positions=target)
    point.time_from_start.sec = 3
    goal.trajectory.points = [point]
    handle = check.future(client.send_goal_async(goal), 10)
    if not handle.accepted:
        raise RuntimeError(f"Rejected direct goal: {controller}")
    try:
        completion = check.future(handle.get_result_async(), 25)
    except BaseException:
        check.future(handle.cancel_goal_async(), 5)
        raise
    if completion.status != 4 or completion.result.error_code != 0:
        raise RuntimeError(f"Failed direct goal: {controller}: {completion}")
    deadline = time.monotonic() + .5
    check.wait(lambda: time.monotonic() >= deadline, 3)
    final = positions(check, names)
    error = max(abs(actual - requested) for actual, requested in zip(final, target))
    row = dict(controller=controller, joints=names, initial_rad=before,
               target_rad=target, final_rad=final, max_endpoint_error_rad=error,
               observed_max_motion_rad=max(abs(a-b) for a,b in zip(final,before)),
               action_status=completion.status, controller_error_code=completion.result.error_code,
               safety=safety, passed=error < .005)
    if not row["passed"]:
        raise RuntimeError(f"Direct goal tracking error: {row}")
    return row


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--head-only", action="store_true")
    mode.add_argument("--arms-only", action="store_true")
    parser.add_argument("--output", default="artifacts/direct_joint_tracking.json")
    args = parser.parse_args()
    rclpy.init()
    check = Check()
    report = dict(tests=[], passed=False)
    try:
        groups = []
        if not args.arms_only:
            groups.append(("head_controller", ["head_1_joint", "head_2_joint"], [.15, .10]))
        if not args.head_only:
            for side in ("left", "right"):
                groups.append((f"arm_{side}_controller",
                               [f"arm_{side}_{i}_joint" for i in range(1, 8)],
                               [0, 0, 0, 0, 0, .08, 0]))
        for controller, names, offsets in groups:
            initial = positions(check, names)
            for sign in (1, -1):
                target = [value + sign * offset for value, offset in zip(initial, offsets)]
                try:
                    audit(check, names, initial, target)
                    break
                except RuntimeError:
                    if sign == -1:
                        raise
            outward = command(check, controller, names, target)
            requested_motion = max(abs(a-b) for a,b in zip(initial,target))
            if outward["observed_max_motion_rad"] < .8 * requested_motion:
                raise RuntimeError("Controller reported success without expected actual motion")
            report["tests"].append(outward)
            report["tests"].append(command(check, controller, names, initial))
            print(f"{controller}: actual outward and return tracking passed", flush=True)
        report["passed"] = True
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n")
        check.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
