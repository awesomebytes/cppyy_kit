"""ROS 2 actuator and state adapter for the TIAGo PRO playground.

Run in the simulator environment with ROS_DOMAIN_ID=77. Positions use metres
and radians. Base poses and controller base velocities use world coordinates.
"""

from __future__ import annotations

import math
import json
import select
import subprocess
import sys
import os
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_config import GRASP_FRAMES, JOINT_GROUPS, SIDES


OBJECTS = {
    "red_cube": (-2.85, 2.23, 1.0455),
    "blue_cylinder": (-2.65728, 2.23, 1.058),
    "yellow_cube": (-2.46, 2.23, 1.048),
}


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Robot:
    """Publish short native trajectories; expose measured ROS feedback."""

    def __init__(self, max_state_age=2.0):
        os.environ.setdefault("ROS_DOMAIN_ID", "77")
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            from rclpy.signals import SignalHandlerOptions

            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("pick_place_robot")
        self._positions = {}
        self._commanded = False
        self._base = None
        self._stamp = None
        self._joint_received = None
        self._odom_received = None
        self.max_state_age = float(max_state_age)
        self._subscriptions = [
            self.node.create_subscription(
                JointState, "/joint_states", self._joint_state, qos_profile_sensor_data
            ),
            self.node.create_subscription(
                Odometry,
                "/mobile_base_controller/odom",
                self._odometry,
                qos_profile_sensor_data,
            ),
        ]
        self.groups = JOINT_GROUPS
        self.publishers = {
            group: self.node.create_publisher(
                JointTrajectory, f"/{group}_controller/joint_trajectory", 1
            )
            for group in self.groups
        }
        self.base_publisher = self.node.create_publisher(
            Twist, "/mobile_base_controller/cmd_vel_unstamped", 1
        )
        self.grippers = {
            side: ActionClient(
                self.node, GripperCommand, f"/gripper_{side}_controller/gripper_cmd"
            )
            for side in SIDES
        }
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self._gripper_goals = []
        self._gz = None
        self._gz_service = None
        self._object_poses = {}
        self._object_received = None
        self._attachments = {}
        self._attachment_update = 0.0

    def _joint_state(self, message):
        self._positions.update(zip(message.name, message.position))
        self._stamp = message.header.stamp
        self._joint_received = time.monotonic()

    def _odometry(self, message):
        self._odom_received = time.monotonic()
        p = message.pose.pose
        self._base = (p.position.x, p.position.y, _yaw(p.orientation))

    def spin(self, timeout=0.0):
        rclpy.spin_once(self.node, timeout_sec=timeout)
        if self._attachments and time.monotonic() - self._attachment_update > 0.08:
            self._attachment_update = time.monotonic()
            for name, side in tuple(self._attachments.items()):
                xyz, _ = self.frame_pose(GRASP_FRAMES[side], "odom")
                self.set_object_pose(name, xyz)

    def wait_ready(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.05)
            if self._positions and self._base is not None:
                return self.state()
        raise TimeoutError("No joint state and odometry received from simulator")

    def state(self):
        if (
            self._joint_received is None
            or time.monotonic() - self._joint_received > self.max_state_age
        ):
            raise RuntimeError("Joint state feedback is stale or unavailable")
        return dict(self._positions)

    def base_pose(self):
        if (
            self._odom_received is None
            or time.monotonic() - self._odom_received > self.max_state_age
        ):
            raise RuntimeError("Odometry feedback is stale or unavailable")
        return self._base

    def command_joints(self, positions, duration=0.15):
        """Send grouped joint targets with a positive trajectory duration."""
        self._commanded = True
        for group, names in self.groups.items():
            selected = [name for name in names if name in positions]
            if not selected:
                continue
            # Controllers require all configured joints unless partial goals are enabled.
            values = [
                float(positions.get(name, self._positions.get(name, 0.0)))
                for name in names
            ]
            point = JointTrajectoryPoint(positions=values)
            seconds = max(0.02, float(duration))
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
            message = JointTrajectory(joint_names=names, points=[point])
            self.publishers[group].publish(message)

    def command_arm(self, side, positions, duration=0.15):
        self.command_joints(dict(zip(self.groups[f"arm_{side}"], positions)), duration)

    def command(self, command, duration=0.15):
        self.state()
        self.base_pose()
        self.command_joints(command.joints, duration)
        self.command_base(*command.base_velocity)

    def command_base(self, vx=0.0, vy=0.0, wz=0.0):
        """Convert world XY velocity to the base frame before publishing."""
        self._commanded = True
        yaw = self.base_pose()[2] if vx or vy else 0.0
        message = Twist()
        message.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        message.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        message.angular.z = float(wz)
        self.base_publisher.publish(message)

    def gripper(self, side, opening=0.07, effort=10.0, wait=False, timeout=8.0):
        """Command leader displacement (0 closed, 0.07 m open).

        A contact grasp can stall before reaching zero. Inspect finger feedback
        and object motion rather than treating action success as grasp evidence.
        """
        self._commanded = True
        client = self.grippers[side]
        if not client.wait_for_server(timeout_sec=timeout):
            raise TimeoutError(f"{side} gripper action is unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = float(opening)
        goal.command.max_effort = float(effort)
        future = client.send_goal_async(goal)
        self._gripper_goals.append(future)
        if wait:
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                self.spin(0.02)
            if not future.done():
                raise TimeoutError("Gripper goal acknowledgement timed out")
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("Gripper goal rejected")
            result = handle.get_result_async()
            while not result.done() and time.monotonic() < deadline:
                self.spin(0.02)
            return result.result() if result.done() else None
        return future

    def frame_pose(self, frame, reference="base_link"):
        transform = self.tf.lookup_transform(
            reference, frame, rclpy.time.Time()
        ).transform
        return (
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def _gazebo(self):
        if self._gz is None:
            os.environ.setdefault("GZ_PARTITION", "tiago_pro_jazzy")
            from gz.transport13 import Node as GazeboNode
            from gz.msgs10.pose_v_pb2 import Pose_V

            self._gz = GazeboNode()
            self._gz.subscribe(
                Pose_V, "/world/default/dynamic_pose/info", self._gazebo_poses
            )
        return self._gz

    def _gazebo_poses(self, message):
        self._object_received = time.monotonic()
        # Model and link share a name. The model has the smaller entity ID.
        models = {}
        for pose in message.pose:
            if pose.name in OBJECTS and (
                pose.name not in models or pose.id < models[pose.name].id
            ):
                models[pose.name] = pose
        for name, pose in models.items():
            q = pose.orientation
            x, y, z = OBJECTS[name]
            tx, ty, tz = (
                2 * (q.y * z - q.z * y),
                2 * (q.z * x - q.x * z),
                2 * (q.x * y - q.y * x),
            )
            self._object_poses[name] = (
                pose.position.x + x + q.w * tx + q.y * tz - q.z * ty,
                pose.position.y + y + q.w * ty + q.z * tx - q.x * tz,
                pose.position.z + z + q.w * tz + q.x * ty - q.y * tx,
            )

    def object_position(self, name):
        """World object centre from Gazebo, for validation only."""
        self._gazebo()
        if (
            self._object_received is None
            or time.monotonic() - self._object_received > self.max_state_age
        ):
            return None
        return self._object_poses.get(name)

    def set_object_pose(self, name, xyz):
        """Explicit simulator assistance, not evidence of a contact grasp.

        Keep Gazebo service transport in its own process. Loading its C++
        dependencies beside Cling can make request dispatch fail silently.
        """
        if self._gz_service is None or self._gz_service.poll() is not None:
            code = """import sys,json
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
node=Node()
for line in sys.stdin:
    value=json.loads(line)
    pose=Pose(name=value[0])
    pose.position.x,pose.position.y,pose.position.z=value[1:]
    pose.orientation.w=1
    success,result=node.request('/world/default/set_pose',pose,Pose,Boolean,1500)
    print(json.dumps(bool(success and result.data)),flush=True)
"""
            environment = dict(os.environ, GZ_PARTITION="tiago_pro_jazzy")
            self._gz_service = subprocess.Popen(
                [sys.executable, "-u", "-c", code],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True,
                env=environment,
            )
        position = [float(a) - b for a, b in zip(xyz, OBJECTS[name])]
        self._gz_service.stdin.write(json.dumps([name, *position]) + "\n")
        self._gz_service.stdin.flush()
        readable, _, _ = select.select([self._gz_service.stdout], [], [], 3.0)
        if not readable or self._gz_service.stdout.readline().strip() != "true":
            raise RuntimeError("Gazebo set_pose failed")

    def attach(self, name, side):
        """Enable disclosed pose-following assistance while spin() is called."""
        self._gazebo()
        self._attachments[name] = side
        self._attachment_update = 0.0
        return True

    def detach(self, name):
        self._attachments.pop(name, None)
        return True

    def close(self):
        if rclpy.ok() and self._commanded:
            self.base_publisher.publish(Twist())
        if self._gz_service is not None:
            self._gz_service.terminate()
            self._gz_service.wait(timeout=3)
            self._gz_service = None
        self.node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()

    def __enter__(self):
        self.wait_ready()
        return self

    def __exit__(self, *_):
        self.close()
