"""TIAGo PRO joints, task frames and default servo costs.

Base translation uses metres. Joint angles and base yaw use radians.
"""

SIDES = ("left", "right")
ARM_JOINT_GROUPS = {
    side: tuple(f"arm_{side}_{index}_joint" for index in range(1, 8)) for side in SIDES
}
ARM_JOINTS = tuple(name for joints in ARM_JOINT_GROUPS.values() for name in joints)
GRIPPER_JOINTS = {side: f"gripper_{side}_finger_joint" for side in SIDES}
TORSO_JOINT = "torso_lift_joint"
HEAD_JOINTS = ("head_1_joint", "head_2_joint")
BASE_JOINTS = ("base_x", "base_y", "base_yaw")
JOINT_GROUPS = {
    **{f"arm_{side}": joints for side, joints in ARM_JOINT_GROUPS.items()},
    "head": HEAD_JOINTS,
    "torso": (TORSO_JOINT,),
}

GRASP_FRAMES = {side: f"gripper_{side}_grasping_link" for side in SIDES}
ELBOW_FRAMES = {side: f"arm_{side}_4_link" for side in SIDES}
GAZE_FRAME = "head_front_camera_color_optical_frame"
ARM_SEGMENT_FRAMES = {
    side: tuple(f"arm_{side}_{index}_link" for index in range(1, 8))
    + (GRASP_FRAMES[side],)
    for side in SIDES
}
DEFAULT_JOINT_COSTS = {
    **dict.fromkeys(ARM_JOINTS + HEAD_JOINTS, 1.0),
    TORSO_JOINT: 15.0,
    **dict.fromkeys(BASE_JOINTS, 100.0),
}
BASE_SPEED_LIMITS = dict(zip(BASE_JOINTS, (0.12, 0.12, 0.2), strict=True))


def active_joint_names(allow_base=True):
    """Return the servo coordinate order, with optional virtual base coordinates."""
    names = list(ARM_JOINTS + (TORSO_JOINT,) + HEAD_JOINTS)
    if allow_base:
        names.extend(BASE_JOINTS)
    return names
