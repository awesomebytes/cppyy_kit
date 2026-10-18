"""Read actual arm/hand meshes and evaluate their bounds through native ROS FK."""
import math
import copy
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from moveit_msgs.srv import GetPositionFK


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rpy_matrix(text):
    r, p, y = map(float, text.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


class Geometry:
    def __init__(self, check):
        self.check = check
        root = Path(__file__).resolve().parent.parent
        model = ET.parse(Path(__file__).parent / "generated/robot.urdf")
        self.parent_joints = {joint.find("child").get("link"): joint for joint in model.findall("joint")}
        self.mimics = [(joint.get("name"), joint.find("mimic").get("joint"),
                        float(joint.find("mimic").get("multiplier", "1")),
                        float(joint.find("mimic").get("offset", "0")))
                       for joint in model.findall("joint") if joint.find("mimic") is not None]
        self.shapes = {}
        for link in model.findall("link"):
            name = link.get("name")
            if not name.startswith(("arm_", "gripper_", "wrist_")):
                continue
            shapes = []
            for visual in link.findall("visual") + link.findall("collision"):
                geometry = visual.find("geometry")
                origin = visual.find("origin")
                rotation = np.eye(3) if origin is None else rpy_matrix(origin.get("rpy", "0 0 0"))
                offset = np.zeros(3) if origin is None else np.array(list(map(float, origin.get("xyz", "0 0 0").split())))
                mesh = geometry.find("mesh")
                if mesh is not None:
                    uri = mesh.get("filename")
                    if uri.startswith("package://"):
                        package, relative = uri[10:].split("/", 1)
                        path = root / "generated/prefix/share" / package / relative.lstrip("/")
                    else:
                        path = Path(uri.removeprefix("file://"))
                    data = path.read_bytes()
                    count = struct.unpack_from("<I", data, 80)[0]
                    if len(data) != 84 + 50 * count:
                        raise RuntimeError("Expected pinned binary STL: " + str(path))
                    vertices = np.array([record[3:12] for record in struct.iter_unpack("<12fH", data[84:])]).reshape(-1, 3)
                    vertices *= np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
                    shapes.append((vertices @ rotation.T + offset, 0.0))
                elif geometry.find("sphere") is not None:
                    shapes.append((offset.reshape(1, 3), float(geometry.find("sphere").get("radius"))))
                elif geometry.find("box") is not None:
                    half = np.array(list(map(float, geometry.find("box").get("size").split()))) / 2
                    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
                    shapes.append((corners @ rotation.T + offset, 0.0))
                elif geometry.find("cylinder") is not None:
                    cylinder = geometry.find("cylinder")
                    radius = float(cylinder.get("radius"))
                    half = np.array([radius, radius, float(cylinder.get("length")) / 2])
                    # Enclosing corners give a conservative clearance bound.
                    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
                    shapes.append((corners @ rotation.T + offset, 0.0))
                else:
                    raise RuntimeError("Unsupported arm/hand geometry in " + name)
            if shapes:
                self.shapes[name] = shapes

    def bounds(self, state):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_footprint"
        request.fk_link_names = list(self.shapes)
        request.robot_state = state
        response = self.check.future(self.check.fk.call_async(request))
        if response.error_code.val != 1:
            raise RuntimeError("Native FK failed while checking hand geometry")
        result = {}
        for name, stamped in zip(response.fk_link_names, response.pose_stamped):
            pose = stamped.pose
            rotation = quaternion_matrix(pose.orientation)
            translation = np.array([pose.position.x, pose.position.y, pose.position.z])
            minima, maxima = [], []
            for points, radius in self.shapes[name]:
                world = points @ rotation.T + translation
                minima.append(world.min(axis=0) - radius)
                maxima.append(world.max(axis=0) + radius)
            result[name] = {"min": np.min(minima, axis=0).tolist(), "max": np.max(maxima, axis=0).tolist()}
        return result

    def clearance(self, state):
        bounds = self.bounds(state)
        return {"minimum_arm_hand_z_m": min(value["min"][2] for value in bounds.values()),
                "minimum_gripper_z_m": min(value["min"][2] for name, value in bounds.items()
                                           if name.startswith("gripper_")),
                "link_bounds": bounds}

    def with_opening(self, state, opening):
        result = copy.deepcopy(state)
        values = dict(zip(result.joint_state.name, result.joint_state.position))
        for side in ("left", "right"):
            values[f"gripper_{side}_finger_joint"] = float(opening)
        for name, parent, multiplier, offset in self.mimics:
            if parent in values:
                values[name] = values[parent] * multiplier + offset
        result.joint_state.name = list(values)
        result.joint_state.position = list(values.values())
        return result

    def full_opening_floor_bound(self, state):
        """Conservative hand reach sphere covers every allowed finger angle/opening."""
        result = {}
        for side in ("left", "right"):
            base = f"gripper_{side}_base_link"
            base_pose = self.check.pose(state, base)
            radius = 0.0
            for name, shapes in self.shapes.items():
                if not name.startswith(f"gripper_{side}_"):
                    continue
                reach = 0.0
                current = name
                while current != base:
                    joint = self.parent_joints[current]
                    origin = joint.find("origin")
                    if origin is not None:
                        reach += np.linalg.norm(list(map(float, origin.get("xyz", "0 0 0").split())))
                    if joint.get("type") == "prismatic":
                        limit = joint.find("limit")
                        axis = joint.find("axis")
                        reach += max(abs(float(limit.get("lower"))), abs(float(limit.get("upper")))) * np.linalg.norm(
                            list(map(float, axis.get("xyz", "1 0 0").split())))
                    current = joint.find("parent").get("link")
                for points, sphere_radius in shapes:
                    radius = max(radius, reach + float(np.linalg.norm(points, axis=1).max()) + sphere_radius)
            result[side] = {"base_height_m": base_pose.position.z,
                            "all_openings_enclosing_radius_m": radius,
                            "all_openings_floor_clearance_bound_m": base_pose.position.z - radius}
        return result
