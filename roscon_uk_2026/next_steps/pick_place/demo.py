"""Select a colored 3D object and run the measured-feedback pick/place sequence."""

from __future__ import annotations

import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import asdict
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

from robot_config import GRASP_FRAMES, GRIPPER_JOINTS

HERE = Path(__file__).resolve().parent


@contextmanager
def deployment(mode):
    if mode == "composed":
        from pipeline import PerceptionPipeline

        with PerceptionPipeline() as pipeline:
            yield pipeline
    elif mode == "distributed":
        processes = []
        try:
            for stage in ("filter", "cluster", "image"):
                processes.append(
                    subprocess.Popen(
                        [sys.executable, str(HERE / "pipeline.py"), "--stage", stage],
                        start_new_session=True,
                    )
                )
            yield processes
        finally:
            for process in processes:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for process in processes:
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    # Compilation can outlast cooperative cancellation. Reap the
                    # owned process group, including its compiler subprocesses.
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
    else:
        yield None


class Observations:
    """Match measured cloud centres to color silhouettes at the image timestamp."""

    def __init__(self, robot):
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data

        self.robot = robot
        self.cloud = self.image = self.camera = None
        self.subscriptions = [
            robot.node.create_subscription(
                String, "/pick_place/objects", self._cloud, 1
            ),
            robot.node.create_subscription(
                String, "/pick_place/image_objects", self._image, 1
            ),
            robot.node.create_subscription(
                CameraInfo,
                "/head_front_camera/color/camera_info",
                self._camera,
                qos_profile_sensor_data,
            ),
        ]

    def _cloud(self, message):
        self.cloud = (json.loads(message.data), time.monotonic())

    def _image(self, message):
        self.image = (json.loads(message.data), time.monotonic())

    def _camera(self, message):
        self.camera = message

    def select(self, color, timeout=90):
        from builtin_interfaces.msg import Time
        from rclpy.time import Time as RosTime
        from controller import _rotation

        deadline = time.monotonic() + timeout
        reason = "waiting for cloud, image and calibration"
        while time.monotonic() < deadline:
            self.robot.spin(0.02)
            if self.cloud is None or self.image is None or self.camera is None:
                continue
            cloud, received_cloud = self.cloud
            image, received_image = self.image
            if time.monotonic() - min(received_cloud, received_image) > 3:
                reason = "perception outputs are stale"
                continue
            cloud_stamp = cloud["stamp_sec"] + cloud["stamp_nanosec"] * 1e-9
            image_stamp = image["stamp_sec"] + image["stamp_nanosec"] * 1e-9
            if abs(cloud_stamp - image_stamp) > 0.3:
                reason = "cloud and image timestamps differ by more than 0.3 s"
                continue
            if not cloud.get("table_found"):
                reason = "no table plane found"
                continue
            stamp = RosTime.from_msg(
                Time(sec=image["stamp_sec"], nanosec=image["stamp_nanosec"])
            )
            try:
                tf = self.robot.tf.lookup_transform(
                    image["frame_id"], cloud["frame_id"], stamp
                ).transform
            except Exception as error:
                reason = str(error)
                continue
            q, t = tf.rotation, tf.translation
            rotation = _rotation((q.x, q.y, q.z, q.w))
            translation = np.array((t.x, t.y, t.z))
            k = self.camera.k
            candidates = []
            for obj in cloud["objects"]:
                point = rotation @ np.asarray(obj["center"]) + translation
                if point[2] <= 0:
                    continue
                u, v = (
                    k[0] * point[0] / point[2] + k[2],
                    k[4] * point[1] / point[2] + k[5],
                )
                for label in image["objects"]:
                    x, y, width, height = label["bbox"]
                    if (
                        label["color"] == color
                        and x - 5 <= u <= x + width + 5
                        and y - 5 <= v <= y + height + 5
                    ):
                        score = ((u - x - width / 2) / width) ** 2 + (
                            (v - y - height / 2) / height
                        ) ** 2
                        candidates.append(
                            (
                                score,
                                dict(
                                    obj,
                                    color=color,
                                    shape=label["shape"],
                                    table_height=cloud["table_height"],
                                    frame_id=cloud["frame_id"],
                                    pixel=[float(u), float(v)],
                                    image_bbox=label["bbox"],
                                    name={
                                        "red": "red_cube",
                                        "blue": "blue_cylinder",
                                        "yellow": "yellow_cube",
                                    }[color],
                                ),
                            )
                        )
            if candidates:
                return min(candidates, key=lambda item: item[0])[1]
            reason = f"no {color} image silhouette matched a 3D cluster"
        raise TimeoutError(reason)


class Servo:
    def __init__(self, robot, controller, timeout=75):
        self.robot, self.controller, self.timeout = robot, controller, timeout
        self.moves = []

    def _target(self, link, position, orientation, position_tolerance=0.01):
        from controller import _rotation, _log_rotation

        self.controller.set_goal(link, position, orientation)
        deadline = time.monotonic() + self.timeout
        cycles = 0
        started = time.monotonic()
        next_report = started + 5.0
        while time.monotonic() < deadline:
            tick = time.monotonic()
            # Drain queued state and TF before calculating a command.
            for _ in range(8):
                self.robot.spin(0.0)
            state, base = self.robot.state(), self.robot.base_pose()
            pose = self.controller.pose(link, state, base)
            error = float(np.linalg.norm(pose[:3, 3] - position))
            angle = (
                0.0
                if orientation is None
                else float(
                    np.linalg.norm(
                        _log_rotation(_rotation(orientation) @ pose[:3, :3].T)
                    )
                )
            )
            if tick >= next_report:
                print(
                    json.dumps(
                        {
                            "event": "servo",
                            "link": link,
                            "position_error_m": error,
                            "orientation_error_rad": angle,
                            "position": pose[:3, 3].tolist(),
                        }
                    ),
                    flush=True,
                )
                next_report = tick + 5.0
            if error < position_tolerance and angle < 0.10:
                self.moves.append(
                    {
                        "link": link,
                        "position_error_m": error,
                        "orientation_error_rad": angle,
                        "cycles": cycles,
                        "wall_s": time.monotonic() - started,
                    }
                )
                self.robot.command_base()
                return True
            command = self.controller.step(state, base, dt=0.05)
            if command.diagnostics.get("solver_failure"):
                self.stop()
                raise RuntimeError(command.diagnostics["solver_failure"])
            self.robot.command(command, duration=0.05)
            cycles += 1
            while time.monotonic() - tick < 0.05:
                self.robot.spin(0.002)
        self.robot.command_base()
        self.moves.append(
            {
                "link": link,
                "position_error_m": error,
                "orientation_error_rad": angle,
                "cycles": cycles,
                "wall_s": time.monotonic() - started,
                "timeout": True,
            }
        )
        return False

    def move(self, link, position, orientation):
        from manipulation import _orientation

        pose = self.controller.pose(link, self.robot.state(), self.robot.base_pose())
        start = pose[:3, 3].copy()
        first = np.asarray(_orientation(pose[:3, 0], pose[:3, 1]))
        last = np.asarray(orientation, dtype=float)
        last /= np.linalg.norm(last)
        if np.dot(first, last) < 0:
            last = -last
        angle = 2 * np.arccos(np.clip(np.dot(first, last), -1.0, 1.0))
        count = max(
            1,
            int(np.ceil(np.linalg.norm(np.asarray(position) - start) / 0.04)),
            int(np.ceil(angle / 0.15)),
        )
        for fraction in np.linspace(1 / count, 1.0, count):
            quaternion = (1 - fraction) * first + fraction * last
            quaternion /= np.linalg.norm(quaternion)
            point = (1 - fraction) * start + fraction * np.asarray(position)
            tolerance = 0.01 if fraction == 1.0 else 0.025
            if not self._target(link, point, quaternion, tolerance):
                return False
        return True

    def prepare(self, side, table_height, clear_body=False):
        """Orient before the table, then lift the downward-facing hand."""
        from manipulation import _orientation

        link = GRASP_FRAMES[side]
        pose = self.controller.pose(link, self.robot.state(), self.robot.base_pose())
        near = pose[:3, 3].copy()
        near[2] = table_height - 0.02
        down = _orientation((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
        timeout = self.timeout
        self.timeout = max(timeout, 120.0)
        try:
            if clear_body:
                # Give the wrist space to turn without crossing the torso.
                # Offsets are in the base frame, with left/right mirrored.
                yaw = self.robot.base_pose()[2]
                forward, lateral = 0.18, 0.15 if side == "left" else -0.15
                outward = pose[:3, 3] + np.array(
                    [
                        np.cos(yaw) * forward - np.sin(yaw) * lateral,
                        np.sin(yaw) * forward + np.cos(yaw) * lateral,
                        0.065,
                    ]
                )
                initial_orientation = _orientation(pose[:3, 0], pose[:3, 1])
                if not self._target(link, outward, initial_orientation):
                    return False
                if not self._target(link, outward, down):
                    return False
            elif not self._target(link, near, down):
                return False
            near[2] = table_height + 0.18
            return self._target(link, near, down)
        finally:
            self.timeout = timeout

    def stop(self):
        self.robot.command_base()
        try:
            self.robot.command_joints(self.robot.state(), duration=0.1)
        except RuntimeError:
            pass  # Stale feedback must not become a new position target.

    def gripper(self, side, opening):
        result = self.robot.gripper(
            side, opening, wait=True, timeout=15 if opening == 0.0 else 8
        )
        # A blocked close is expected for a contact grasp. The manipulation
        # sequence checks measured object lift before declaring success.
        if result is None:
            return False
        from action_msgs.msg import GoalStatus

        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.reached_goal:
            return True
        finger = self.robot.state().get(GRIPPER_JOINTS[side], 0.0)
        return (
            opening == 0.0
            and result.result.stalled
            and 0.002 < finger < 0.068
            and result.status
            in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--deployment",
        choices=("composed", "distributed", "external"),
        default="composed",
    )
    parser.add_argument(
        "--controller",
        choices=("custom", "placo"),
        default="custom",
        help="whole-body solver; Placo requires the pick-place-placo environment",
    )
    parser.add_argument("--color", choices=("red", "blue", "yellow"), default="red")
    parser.add_argument(
        "--observe",
        action="store_true",
        help="report the selected object without moving",
    )
    parser.add_argument(
        "--allow-attachment",
        action="store_true",
        help="disclose and use pose-following if contact fails",
    )
    parser.add_argument(
        "--handover",
        action="store_true",
        help="experimental arm transfer; not a validated demo",
    )
    parser.add_argument("--no-base", action="store_true")
    parser.add_argument(
        "--base-y-max",
        type=float,
        default=1.60,
        help="scene-specific upper bound on base world Y, metres",
    )
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=HERE.parent / "tiago_pro_jazzy/generated/robot_description.urdf",
    )
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    # Match the playground regardless of the root workspace activation default.
    os.environ["ROS_DOMAIN_ID"] = "77"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["GZ_PARTITION"] = "tiago_pro_jazzy"
    from robot import Robot
    from manipulation import PickPlace, PickConfig

    with ExitStack() as stack:
        stack.enter_context(deployment(args.deployment))
        robot = stack.enter_context(Robot())
        observations = Observations(robot)
        detection = observations.select(args.color)
        print(json.dumps({"event": "selected", **detection}), flush=True)
        if args.observe:
            report = {"selected": detection}
        else:
            if args.controller == "placo":
                from placo_controller import (
                    PlacoWholeBodyController as WholeBodyController,
                )
            else:
                from controller import WholeBodyController
            controller = WholeBodyController(
                args.urdf.read_text(), allow_base=not args.no_base
            )
            if not args.no_base:
                controller.set_base_bounds("y", -np.inf, args.base_y_max)
            servo = Servo(robot, controller)
            stack.callback(servo.stop)
            # Gazebo poses validate success. Reach targets come from perception.
            deadline = time.monotonic() + 5
            while (
                robot.object_position(detection["name"]) is None
                and time.monotonic() < deadline
            ):
                robot.spin(0.05)
            if not servo.prepare(
                args.side,
                detection["table_height"],
                clear_body=args.controller == "placo",
            ):
                servo.stop()
                raise RuntimeError("Initial lift/orientation did not converge")
            if args.handover and not servo.prepare(
                "right" if args.side == "left" else "left",
                detection["table_height"],
                clear_body=args.controller == "placo",
            ):
                servo.stop()
                raise RuntimeError("Receiver preparation did not converge")
            task = PickPlace(
                servo.move,
                servo.gripper,
                observe_object=robot.object_position,
                attach=robot.attach,
                detach=robot.detach,
                stop=servo.stop,
                config=PickConfig(
                    side=args.side,
                    allow_attachment=args.allow_attachment,
                    base_yaw=robot.base_pose()[2],
                ),
            )
            result = task.run(detection, handover=args.handover)
            report = {
                "controller": args.controller,
                "selected": detection,
                **asdict(result),
                "moves": servo.moves,
            }
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
        return 0 if args.observe or report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
