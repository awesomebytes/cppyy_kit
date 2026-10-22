"""Bounded pick/place stages driven by measured world-frame detections.

The move callback must wait for measured position and orientation convergence.
Object poses are used only to validate contact and release, never to plan a reach.
"""
from dataclasses import dataclass, field
import math
import numpy as np


def _xyz(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("Expected three finite coordinates in metres")
    return value.copy()


def _field(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def _orientation(approach, closing):
    """Quaternion XYZW for grasp-frame X approach and Y finger separation."""
    x = _xyz(approach)
    if np.linalg.norm(x) < 1e-8:
        raise ValueError("Approach axis must be nonzero")
    x /= np.linalg.norm(x)
    y = _xyz(closing)
    y -= x * np.dot(x, y)
    if np.linalg.norm(y) < 1e-8:
        raise ValueError("Closing axis must not be parallel to the approach axis")
    y /= np.linalg.norm(y)
    r = np.column_stack((x, y, np.cross(x, y)))
    # Select the largest quaternion component to remain stable at half turns.
    candidates = np.array([1+r[0,0]-r[1,1]-r[2,2],
                           1-r[0,0]+r[1,1]-r[2,2],
                           1-r[0,0]-r[1,1]+r[2,2], 1+np.trace(r)])
    i = int(np.argmax(candidates))
    q = np.zeros(4)
    q[i] = .5 * math.sqrt(max(candidates[i], 0.))
    d = 4*q[i]
    if i == 3:
        q[:3] = [(r[2,1]-r[1,2])/d, (r[0,2]-r[2,0])/d, (r[1,0]-r[0,1])/d]
    else:
        j, k = (i+1)%3, (i+2)%3
        q[j], q[k] = (r[i,j]+r[j,i])/d, (r[i,k]+r[k,i])/d
        q[3] = (r[k,j]-r[j,k])/d
    return tuple(float(v) for v in q)


@dataclass
class PickConfig:
    side: str = "left"
    approach_height: float = .15
    lift_height: float = .12
    grasp_offset: float = .008
    place_offset: tuple = (-.16, 0., 0.)
    open_position: float = .07
    close_position: float = 0.
    minimum_lift: float = .06
    placement_tolerance: float = .06
    base_yaw: float = 0.
    closing_axis: tuple = (1., 0., 0.)
    allow_attachment: bool = False


@dataclass
class PickResult:
    success: bool = False
    mode: str = "contact"
    stage: str = "initial"
    message: str = ""
    handover: bool = False
    holding_side: str | None = None
    attachment_active: bool = False
    measurements: dict = field(default_factory=dict)


class PickPlace:
    """Sequence callbacks use world metres, XYZW quaternions and gripper joint metres.

    ``move(link, position, orientation)`` and ``gripper(side, position)`` return
    bool after completing the command. ``observe_object(name)`` returns a world
    XYZ position. Optional ``attach(name, side)`` and ``detach(name)`` callbacks
    implement an explicitly enabled simulator attachment fallback.
    """

    def __init__(self, move, gripper, *, observe_object=None, attach=None,
                 detach=None, stop=None, report=print, config=None):
        self.move, self.gripper = move, gripper
        self.observe_object, self.attach, self.detach = observe_object, attach, detach
        self.stop, self.report = stop, report
        self.config = config or PickConfig()
        if self.config.side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.result = PickResult()

    def _stage(self, stage):
        self.result.stage = stage
        self.report(f"manipulation: {stage}")

    def _move(self, side, position, orientation, stage):
        self._stage(stage)
        if not self.move(f"gripper_{side}_grasping_link", tuple(position), orientation):
            raise RuntimeError(f"Measured reach failed at {stage}")

    def _gripper(self, side, position, stage):
        self._stage(stage)
        if not self.gripper(side, position):
            raise RuntimeError(f"Gripper command failed at {stage}")

    def _observe(self, name):
        if self.observe_object is None or not name:
            raise RuntimeError("Object motion validation requires a named object observer")
        value = self.observe_object(name)
        if value is None:
            raise RuntimeError(f"No measured simulator pose for {name}")
        return _xyz(value)

    def run(self, detection, place_position=None, handover=False):
        """Pick a detection with center/lower/upper and optional table_height.

        The name identifies a simulator object for validation. It must be matched
        from perception before this call. Place position is the object centre.
        Failure stops commands and preserves the closed gripper if holding.
        """
        self.result = PickResult()
        c = self.config
        side = c.side
        attached = False
        try:
            center = _xyz(_field(detection, "center"))
            lower = _field(detection, "lower", _field(detection, "minimum"))
            upper = _field(detection, "upper", _field(detection, "maximum"))
            if lower is not None and upper is not None:
                lower, upper = _xyz(lower), _xyz(upper)
                if np.any(upper <= lower):
                    raise ValueError("Detection bounds must have positive dimensions")
                center = .5 * (lower + upper)
                table = _field(detection, "table_height")
                if table is not None:
                    if not np.isfinite(table) or table >= upper[2]:
                        raise ValueError("Table height must be finite and below object top")
                    center[2] = .5 * (float(table) + upper[2])
                axis = _xyz(c.closing_axis)
                axis /= max(np.linalg.norm(axis), 1e-12)
                if np.dot(np.abs(axis), upper-lower) > .10:
                    raise ValueError("Detection exceeds the configured gripper width")
            name = _field(detection, "name")
            baseline = self._observe(name)
            destination = center + _xyz(c.place_offset) if place_position is None else _xyz(place_position)
            grasp = center + np.array([0., 0., c.grasp_offset])
            approach = grasp + np.array([0., 0., c.approach_height])
            lifted = grasp + np.array([0., 0., c.lift_height])
            # URDF grasp frame has Ry(-1.57) relative to the gripper base.
            # Its +X follows the fingers. +Y is the finger separation axis.
            down = _orientation((0., 0., -1.), c.closing_axis)
            self._gripper(side, c.open_position, "open")
            self._move(side, approach, down, "approach")
            self._move(side, grasp, down, "descend")
            self._gripper(side, c.close_position, "close")
            self._move(side, lifted, down, "lift")
            raised = self._observe(name)
            self.result.measurements["contact_lift_m"] = float(raised[2]-baseline[2])
            follow_error = float(np.linalg.norm(raised-baseline-np.array([0., 0., c.lift_height])))
            self.result.measurements["contact_follow_error_m"] = follow_error
            if raised[2]-baseline[2] < c.minimum_lift or follow_error > c.placement_tolerance:
                if not c.allow_attachment or self.attach is None or self.detach is None:
                    raise RuntimeError("Contact grasp did not lift the object")
                if np.linalg.norm(raised-baseline) > c.placement_tolerance:
                    raise RuntimeError("Object moved after failed contact; acquire a new detection")
                self.report("manipulation: contact failed; enabling simulated attachment")
                self._move(side, grasp, down, "attachment_return")
                self._gripper(side, c.close_position, "attachment_close")
                if not self.attach(name, side):
                    raise RuntimeError("Simulator attachment failed")
                attached = True
                self.result.mode = "simulated_attachment"
                self.result.attachment_active = True
                self.result.holding_side = side
                self._move(side, lifted, down, "attachment_lift")
                raised = self._observe(name)
                if raised[2]-baseline[2] < c.minimum_lift:
                    raise RuntimeError("Attached object did not follow the lift")
            self.result.holding_side = side
            if handover:
                side, down = self._handover(name, side, lifted, down, attached)
                self.result.handover = True
            place = destination + np.array([0., 0., c.grasp_offset])
            self._move(side, place + np.array([0., 0., c.approach_height]), down, "place_approach")
            self._move(side, place, down, "place_descend")
            self._gripper(side, c.open_position, "release")
            if attached:
                if not self.detach(name):
                    raise RuntimeError("Simulator attachment release failed")
                attached = False
                self.result.attachment_active = False
            self.result.holding_side = None
            self._move(side, place + np.array([0., 0., c.approach_height]), down, "retreat")
            final = self._observe(name)
            distance = float(np.linalg.norm(final-destination))
            self.result.measurements["placement_error_m"] = distance
            if distance > c.placement_tolerance:
                raise RuntimeError("Released object is outside the placement tolerance")
            self.result.success = True
            self.result.message = "Measured object lift and placement completed"
            self._stage("complete")
        except (RuntimeError, ValueError, TimeoutError) as error:
            self.result.message = str(error)
            self.report(f"manipulation stopped: {error}")
        finally:
            if self.stop is not None:
                try:
                    self.stop()
                except Exception as error:
                    self.result.success = False
                    self.result.message = f"Stop command failed: {error}"
                    self.report(self.result.message)
        return self.result

    def _handover(self, name, giver, lifted, down, attached):
        """Attempt opposing side grasps above the source object."""
        c = self.config
        receiver = "right" if giver == "left" else "left"
        left = np.array([-math.sin(c.base_yaw), math.cos(c.base_yaw), 0.])
        direction = -left if giver == "left" else left
        forward = np.array([math.cos(c.base_yaw), math.sin(c.base_yaw), 0.])
        giving = _orientation(direction, forward)
        receiving = _orientation(-direction, (0., 0., 1.))
        transfer = lifted + direction*.12 + np.array([0., 0., .08])
        self._gripper(receiver, c.open_position, "handover_open")
        self._move(giver, transfer, giving, "handover_present")
        self._move(receiver, transfer + direction*.14, receiving, "handover_approach")
        self._move(receiver, transfer, receiving, "handover_receive")
        self._gripper(receiver, c.close_position, "handover_close")
        self._gripper(giver, c.open_position, "handover_release")
        if attached and not self.attach(name, receiver):
            raise RuntimeError("Simulator attachment transfer failed")
        self.result.holding_side = receiver
        self._move(giver, transfer-direction*.16, giving, "handover_giver_retreat")
        self._move(receiver, transfer+np.array([0., 0., .06]), receiving, "handover_lift")
        measured = self._observe(name)
        self.result.measurements["handover_height_m"] = float(measured[2])
        follow_error = float(np.linalg.norm(measured-transfer-np.array([0., 0., .06])))
        self.result.measurements["handover_follow_error_m"] = follow_error
        if follow_error > c.placement_tolerance:
            raise RuntimeError("Object did not remain with the receiving gripper")
        # Return to the top-down pose before the place descent.
        self._move(receiver, transfer+np.array([0., 0., .06]), down, "handover_turn")
        return receiver, down
