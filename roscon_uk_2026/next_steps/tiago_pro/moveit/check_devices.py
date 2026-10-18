"""Verify stock MoveIt plans/execution against real gripper and head controllers."""
import argparse
import json
from pathlib import Path
import time

from action_msgs.msg import GoalStatusArray
from control_msgs.action import GripperCommand
from rclpy.qos import DurabilityPolicy, QoSProfile
import rclpy
import yaml
from moveit_msgs.msg import Constraints, JointConstraint

from check import Check


class Devices(Check):
    def __init__(self):
        super().__init__()
        self.states = yaml.safe_load((Path(__file__).parent / "device_postures.yaml").read_text())
        self.gripper_status = {}
        self.gripper_subscriptions = []
        for side in ("left", "right"):
            topic = f"/gripper_{side}_controller/gripper_cmd/_action/status"
            self.gripper_subscriptions.append(self.node.create_subscription(
                GoalStatusArray, topic, lambda message, key=side: self.gripper_status.update({key: message}),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def gripper_result(self, side, old_ids):
        def new_goal():
            return any(bytes(status.goal_info.goal_id.uuid) not in old_ids
                       for status in self.gripper_status.get(side, GoalStatusArray()).status_list)
        self.wait(new_goal, 5)
        status = next(status for status in reversed(self.gripper_status[side].status_list)
                      if bytes(status.goal_info.goal_id.uuid) not in old_ids)
        service = GripperCommand.Impl.GetResultService
        client = self.node.create_client(service, f"/gripper_{side}_controller/gripper_cmd/_action/get_result")
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError("Native gripper result service is unavailable")
            result = self.future(client.call_async(service.Request(goal_id=status.goal_info.goal_id)), 10)
            if result.status != 4 or not result.result.reached_goal or result.result.stalled:
                raise RuntimeError("Native effort gripper action did not reach its endpoint")
            return {"goal_uuid": bytes(status.goal_info.goal_id.uuid).hex(),
                    "status": result.status, "reached_goal": result.result.reached_goal,
                    "stalled": result.result.stalled, "position_m": result.result.position,
                    "effort_N": result.result.effort}
        finally:
            self.node.destroy_client(client)

    def named(self, group, name, plan_only=False):
        values = self.states[group][name]
        before = self.state()
        before_values = dict(zip(before.joint_state.name, before.joint_state.position))
        target = self.state()
        constraints = Constraints(name=name)
        gripper = group.startswith("gripper_")
        tolerance = 0.00005 if gripper else 0.001
        for joint, value in values.items():
            target.joint_state.position[target.joint_state.name.index(joint)] = value
            constraints.joint_constraints.append(JointConstraint(joint_name=joint, position=value,
                tolerance_above=tolerance, tolerance_below=tolerance, weight=1.0))
        if not self.within_bounds(target) or not self.validity(target).valid:
            raise RuntimeError(f"Named target {group}/{name} violates bounds or collision constraints")
        side = group.removeprefix("gripper_") if gripper else None
        old_ids = {bytes(status.goal_info.goal_id.uuid)
                   for status in self.gripper_status.get(side, GoalStatusArray()).status_list}
        result, endpoint = self.execute(group, constraints, plan_only=plan_only)
        record = {"group": group, "named_state": name, "executed": not plan_only,
                  "moveit_error": result.error_code.val,
                  "planned_joints": list(result.planned_trajectory.joint_trajectory.joint_names),
                  "planned_points": len(result.planned_trajectory.joint_trajectory.points),
                  "floor_audit": self.last_audit}
        if plan_only:
            return record
        if gripper:
            record["native_gripper_action"] = self.gripper_result(side, old_ids)
        # The physical effort controller settles after its action reaches tolerance.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.03)
        actual = dict(zip(self.latest.name, self.latest.position))
        errors = {joint: abs(actual[joint] - value) for joint, value in values.items()}
        maximum_error = max(errors.values())
        allowed = 0.001 if gripper else 0.01
        if maximum_error > allowed:
            raise RuntimeError(f"Actual {group}/{name} endpoint missed: {errors}")
        if not self.validity(self.state()).valid:
            raise RuntimeError("Actual device endpoint is in collision")
        record.update({"requested_joint_values": values, "actual_joint_values": {key: actual[key] for key in values},
                       "maximum_endpoint_error": maximum_error,
                       "maximum_requested_joint_movement": max(abs(actual[key] - before_values[key]) for key in values),
                       "other_arm_maximum_motion_rad": max(abs(actual[key] - value) for key, value in before_values.items()
                                                             if key.startswith(("arm_left_", "arm_right_"))),
                       "final_collision_free": True})
        return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--groups", nargs="+", choices=["gripper_left", "gripper_right", "head"],
                        default=["gripper_left", "gripper_right"])
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "generated/devices.json")
    args = parser.parse_args()
    rclpy.init()
    check = Devices()
    rows = []
    try:
        for group in args.groups:
            names = ["closed", "open"] if group.startswith("gripper_") else ["look_left", "look_right", "center"]
            for name in names:
                row = check.named(group, name, plan_only=args.plan_only)
                rows.append(row)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(rows, indent=2) + "\n")
                print(json.dumps(row), flush=True)
    finally:
        check.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
