#!/usr/bin/env python3
"""Variant c: run an in-process ros2_control loop at the target rate.

The harness calls `rig.update()` once per cycle. This executes the controller manager's
read, update, and write steps and a Python PD controller that implements
`ControllerInterface`. The measured wakeup latency includes the cross-language update
call, controller-manager read and write work, and any interpreter or garbage-collection
pause. Mock hardware is used. This variant raises `ControlUnavailable` when
ros2_control is not installed.
"""
import os

ROS_DOMAIN_ID = os.environ.setdefault("ROS_DOMAIN_ID", "63")

JOINTS = ["joint1", "joint2"]
FWD = "forward_command_controller/ForwardCommandController"


class ControlUnavailable(RuntimeError):
    """ros2_control (the control env) is not installed on this interpreter."""


def have_control():
    return os.path.isdir(
        os.path.join(os.environ.get("CONDA_PREFIX", ""), "include", "controller_manager"))


def _make_pd_class(ck):
    class PyPD(ck.ControllerInterface):
        def __init__(self):
            super().__init__()
            self.kp = 5.0
            self.target = [0.3, -0.2]

        def on_init(self):
            return ck.CallbackReturn.SUCCESS

        def command_interface_configuration(self):
            return ck.interface_config(["%s/position" % j for j in JOINTS])

        def state_interface_configuration(self):
            return ck.interface_config(["%s/position" % j for j in JOINTS])

        def on_configure(self, s):
            return ck.CallbackReturn.SUCCESS

        def on_activate(self, s):
            return ck.CallbackReturn.SUCCESS

        def on_deactivate(self, s):
            return ck.CallbackReturn.SUCCESS

        def update(self, time_, period):
            for i in range(ck.n_command_interfaces(self)):
                p = ck.read_state(self, i)
                ck.write_command(self, i, p + self.kp * (self.target[i] - p) * 0.01)
            return ck.return_type.OK
    return PyPD


class ControlLoop:
    """Own a ControllerManager and injected Python controller. Call ``setup()`` before
    using ``body(i)`` to run one control cycle. Call ``teardown()`` to stop the manager."""

    def __init__(self, rate_hz, controller="python"):
        self.rate_hz = rate_hz
        self.controller = controller
        self.rig = None
        self._ck = None
        self._period = None

    def setup(self):
        if not have_control():
            raise ControlUnavailable("ros2_control headers not found (use the control env)")
        import cppyy
        from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
        import control_kit as ck
        self._ck = ck
        rclcpp = bringup_rclcpp()
        if not rclcpp.ok():
            rclcpp.init()
        ck.bringup_control()
        ck.warmup()
        rig = ck.make_controller_manager(ck.mock_system_urdf(JOINTS),
                                         update_rate=int(self.rate_hz))
        if self.controller == "cpp":
            rig.load_controller("fwd", FWD,
                                parameters={"joints": JOINTS, "interface_name": "position"})
            rig.configure("fwd")
            rig.activate(["fwd"])
        else:
            pd = _make_pd_class(ck)()
            rig.add_python_controller(pd, "pd")
            rig.configure("pd")
            rig.activate(["pd"])
        self.rig = rig
        self._period = cppyy.gbl.rclcpp.Duration.from_seconds(1.0 / self.rate_hz)
        # Warm the per-controller update() call wrapper off the timed loop (GAP #2 in
        # control_kit REPORT: the first cross into update() JITs a ~29 ms wrapper).
        for _ in range(200):
            self.rig.update(self._period)
        return self

    def body(self, i):
        self.rig.update(self._period)

    def teardown(self):
        if self.rig is not None:
            try:
                self.rig._teardown()
            except Exception:
                pass
            self.rig = None
