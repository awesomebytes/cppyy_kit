"""Validate native MoveIt pose planning and real trajectory feedback on both arms."""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint, RobotState)
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from geometry_msgs.msg import Pose
from geometry import Geometry


class FloorClearanceError(RuntimeError):
    """A native plan exists, but its actual geometry fails the floor audit."""


class Check:
    def __init__(self):
        self.node = rclpy.create_node("pro_moveit_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.latest = None
        model = ET.parse(Path(__file__).parent / "generated/robot.urdf")
        self.limits = {joint.get("name"): (float(joint.find("limit").get("lower")),
                                          float(joint.find("limit").get("upper")))
                       for joint in model.findall("joint") if joint.get("type") in ("revolute", "prismatic")}
        self.subscription = self.node.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.valid = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.move = ActionClient(self.node, MoveGroup, "/move_action")
        self.execute_trajectory = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        self.geometry = Geometry(self)
        self.last_audit = None
        for service in (self.fk, self.valid):
            if not service.wait_for_service(timeout_sec=60):
                raise RuntimeError("MoveIt service unavailable: " + service.srv_name)
        if not self.move.wait_for_server(timeout_sec=60):
            raise RuntimeError("MoveGroup action unavailable")
        self.wait(lambda: self.latest is not None, 15)

    def on_state(self, message):
        self.latest = message

    def wait(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError("Timed out awaiting MoveIt or joint feedback")

    def future(self, future, seconds=60):
        self.wait(future.done, seconds)
        return future.result()

    def state(self):
        # Wheel velocity and effort fields may contain NaN; planning consumes position.
        state = RobotState()
        state.joint_state.name = list(self.latest.name)
        state.joint_state.position = list(self.latest.position)
        return state

    def validity(self, state, group=""):
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = group
        return self.future(self.valid.call_async(request))

    def within_bounds(self, state):
        return all(name not in self.limits or self.limits[name][0] - 1e-4 <= value <= self.limits[name][1] + 1e-4
                   for name, value in zip(state.joint_state.name, state.joint_state.position))

    def pose(self, state, link):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_footprint"
        request.fk_link_names = [link]
        request.robot_state = state
        response = self.future(self.fk.call_async(request))
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("Forward kinematics failed: " + str(response.error_code.val))
        return response.pose_stamped[0].pose

    def execute(self, group, constraints, plan_only=False):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        # Audit the returned path before sending any controller command.
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        for attempt in range(1, 4):
            handle = self.future(self.move.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError("MoveGroup goal rejected")
            result = self.future(handle.get_result_async(), 120).result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"MoveGroup failed: {result.error_code.val}")
            trajectory = result.planned_trajectory.joint_trajectory
            if not trajectory.points:
                raise RuntimeError("MoveIt returned no actual trajectory")
            try:
                self.last_audit = self.audit(trajectory)
                self.last_audit["planning_attempt"] = attempt
                break
            except FloorClearanceError as error:
                print(f"Rejected floor clearance on plan {attempt}/3: {error}", flush=True)
                if attempt == 3:
                    raise
        endpoint = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        if plan_only:
            return result, endpoint
        if not self.execute_trajectory.wait_for_server(timeout_sec=30):
            raise RuntimeError("Native ExecuteTrajectory action is unavailable")
        execution_goal = ExecuteTrajectory.Goal(trajectory=result.planned_trajectory)
        execution = self.future(self.execute_trajectory.send_goal_async(execution_goal))
        if not execution.accepted:
            raise RuntimeError("Native trajectory execution rejected")
        try:
            execution_result = self.future(execution.get_result_async(), 180).result
        except TimeoutError:
            self.future(execution.cancel_goal_async(), 10)
            raise
        if execution_result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Native trajectory execution failed: {execution_result.error_code.val}")
        self.wait(lambda: all(abs(dict(zip(self.latest.name, self.latest.position))[name] - value) < 0.025
                              for name, value in endpoint.items()), 10)
        return result, endpoint

    def audit(self, trajectory):
        state = self.state()
        minimum = math.inf
        minimum_hand = math.inf
        for point in trajectory.points:
            for name, value in zip(trajectory.joint_names, point.positions):
                state.joint_state.position[state.joint_state.name.index(name)] = value
            if not self.within_bounds(state):
                raise RuntimeError("Planned trajectory violates a joint limit")
            bounds = self.geometry.clearance(state)
            minimum = min(minimum, bounds["minimum_arm_hand_z_m"])
            minimum_hand = min(minimum_hand, bounds["minimum_gripper_z_m"])
            if minimum < 0.05:
                raise FloorClearanceError(f"Planned arm/hand geometry lacks 5 cm floor clearance: {minimum} m")
        return {"trajectory_points_checked": len(trajectory.points),
                "minimum_arm_hand_z_m": minimum, "minimum_gripper_z_m": minimum_hand,
                "minimum_required_geometry_height_m": 0.05}

    def pose_check(self, side, plan_only=False):
        group = "arm_" + side
        link = group + "_tool_link"
        before = self.state()
        validity = self.validity(before)
        if not validity.valid:
            raise RuntimeError("Current robot is in collision: " + str([
                (contact.contact_body_1, contact.contact_body_2) for contact in validity.contacts]))
        # Create a reachable pose independently by FK of a nearby valid joint state.
        target = copy.deepcopy(before)
        joint = f"arm_{side}_6_joint"
        index = target.joint_state.name.index(joint)
        for increment in (0.16, -0.16, 0.08, -0.08):
            target.joint_state.position[index] = before.joint_state.position[index] + increment
            if self.within_bounds(target) and self.validity(target).valid:
                break
        else:
            raise RuntimeError("No nearby collision-free test target")
        pose = self.pose(target, link)
        constraints = Constraints()
        constraints.name = side + "_reachable_pose"
        position = PositionConstraint()
        position.header.frame_id = "base_footprint"
        position.link_name = link
        position.weight = 1.0
        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.008])
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [region_pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = "base_footprint"
        orientation.link_name = link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.02
        orientation.absolute_y_axis_tolerance = 0.02
        orientation.absolute_z_axis_tolerance = 0.02
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        result, endpoint = self.execute(group, constraints, plan_only=plan_only)
        if plan_only:
            return {"group": group, "tool_link": link, "moveit_error": result.error_code.val,
                    "executed": False,
                    "floor_audit": self.last_audit,
                    "trajectory_points": len(result.planned_trajectory.joint_trajectory.points)}
        after = self.state()
        actual = self.pose(after, link)
        position_error = math.sqrt(sum((getattr(actual.position, axis) - getattr(pose.position, axis)) ** 2
                                       for axis in ("x", "y", "z")))
        dot = sum(getattr(actual.orientation, axis) * getattr(pose.orientation, axis)
                  for axis in ("x", "y", "z", "w"))
        orientation_error = 2 * math.acos(min(1.0, abs(dot)))
        before_values = dict(zip(before.joint_state.name, before.joint_state.position))
        after_values = dict(zip(after.joint_state.name, after.joint_state.position))
        movement = max(abs(after_values[name] - before_values[name]) for name in endpoint)
        endpoint_error = max(abs(after_values[name] - value) for name, value in endpoint.items())
        if position_error > 0.015 or orientation_error > 0.05 or movement < 0.025:
            raise RuntimeError(f"Observed pose/motion did not match: {position_error}, {orientation_error}, {movement}")
        if not self.validity(after).valid:
            raise RuntimeError("Final robot state is in collision")
        return {"group": group, "tool_link": link, "moveit_error": result.error_code.val, "executed": True,
                "floor_audit": self.last_audit,
                "trajectory_points": len(result.planned_trajectory.joint_trajectory.points),
                "max_joint_movement_rad": movement, "max_endpoint_error_rad": endpoint_error,
                "tool_position_error_m": position_error, "tool_orientation_error_rad": orientation_error,
                "final_collision_free": True}

    def posture(self, name, plan_only=False):
        values = yaml.safe_load((Path(__file__).parent / "postures.yaml").read_text())[name]
        group = "both_arms_torso" if "torso_lift_joint" in values else "both_arms"
        constraints = Constraints(name=name)
        target = self.state()
        for arm, joints in values.items():
            if arm == "torso_lift_joint":
                target.joint_state.position[target.joint_state.name.index(arm)] = joints
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=arm, position=joints, tolerance_above=0.001,
                    tolerance_below=0.001, weight=1.0))
                continue
            for index, value in enumerate(joints, 1):
                joint_name = f"{arm}_{index}_joint"
                target.joint_state.position[target.joint_state.name.index(joint_name)] = value
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=joint_name, position=value, tolerance_above=0.005,
                    tolerance_below=0.005, weight=1.0))
        if not self.within_bounds(target):
            raise RuntimeError(f"{name} target violates URDF joint limits")
        if not self.validity(target).valid:
            raise RuntimeError(f"{name} target is not collision-free in current scene")
        openings = self.geometry.full_opening_floor_bound(target)
        if any(value["all_openings_floor_clearance_bound_m"] < 0.05 for value in openings.values()):
            raise RuntimeError(f"{name} hands lack floor clearance over the full opening range")
        result, endpoint = self.execute(group, constraints, plan_only=plan_only)
        if plan_only:
            return {"group": group, "posture": name, "moveit_error": result.error_code.val,
                    "executed": False,
                    "floor_audit": self.last_audit,
                    "all_openings_bounds": openings,
                    "trajectory_points": len(result.planned_trajectory.joint_trajectory.points)}
        actual = self.state()
        positions = dict(zip(actual.joint_state.name, actual.joint_state.position))
        if not self.validity(actual).valid:
            raise RuntimeError(f"Observed {name} posture is in collision")
        return {"group": group, "posture": name, "moveit_error": result.error_code.val, "executed": True,
                "floor_audit": self.last_audit,
                "all_openings_bounds": openings,
                "actual_joint_values": {joint: positions[joint] for joint in endpoint},
                "endpoint_errors": {joint: abs(positions[joint] - value) for joint, value in endpoint.items()},
                "max_endpoint_error_rad": max(abs(positions[joint] - value) for joint, value in endpoint.items()
                                              if joint.startswith("arm_")),
                "torso_endpoint_error_m": abs(positions["torso_lift_joint"] - endpoint["torso_lift_joint"])
                                          if "torso_lift_joint" in endpoint else None,
                "final_collision_free": True}

    def compact(self, plan_only=False):
        return self.posture("compact", plan_only=plan_only)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "generated/check.json")
    parser.add_argument("--compact", action="store_true", help="Finish in the collision-checked compact posture")
    parser.add_argument("--plan-only", action="store_true", help="Check planning without issuing controller commands")
    parser.add_argument("--compact-only", action="store_true", help="Plan and execute only the compact joint posture")
    parser.add_argument("--ready-only", action="store_true", help="Plan the ready arm and torso posture, without separate arm tests")
    args = parser.parse_args()
    rclpy.init()
    check = Check()
    try:
        results = []
        if not args.compact_only and not args.ready_only:
            for side in ("left", "right"):
                result = check.pose_check(side, plan_only=args.plan_only)
                results.append(result)
                print(json.dumps({"completed": result}), flush=True)
        if args.compact or args.compact_only:
            result = check.compact(plan_only=args.plan_only)
            results.append(result)
            print(json.dumps({"completed": result}), flush=True)
        if args.ready_only:
            result = check.posture("ready", plan_only=args.plan_only)
            results.append(result)
            print(json.dumps({"completed": result}), flush=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + "\n")
        print(json.dumps(results, indent=2))
    finally:
        check.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
