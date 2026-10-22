"""Move one simulated arm joint with a Python or C++ control_kit position servo.

The external simulator must inherit the plugin overlay in AMENT_PREFIX_PATH.
Run --prepare-only before starting it. Python configures the controller and
publishes references. The selected control law executes inside Gazebo.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET


def prepare_overlay(path, backend):
    if backend == "python":
        from control_kit.python_control import install_python_controller_plugin

        return install_python_controller_plugin(path)
    from control_kit.native_control import install_native_controller_plugin

    return install_native_controller_plugin(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("native", "python"), default="native")
    parser.add_argument(
        "--overlay", type=Path, default=Path.home() / ".cache/cppyy_kit/control_plugin"
    )
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--offset",
        type=float,
        default=0.04,
        help="Last arm joint displacement, radians",
    )
    parser.add_argument(
        "--duration", type=float, default=6.0, help="Wall seconds per move"
    )
    parser.add_argument(
        "--report", type=Path
    )
    args = parser.parse_args()
    if args.report is None:
        args.report = Path(f"{args.backend}_control_evidence.json")
    if not 0.005 <= abs(args.offset) <= 0.08:
        parser.error("absolute offset must be between 0.005 and 0.08 rad")
    if args.duration < 2.0:
        parser.error("duration must be at least 2 wall seconds")
    args.overlay = args.overlay.resolve()
    if args.prepare_only:
        prepare_overlay(args.overlay, args.backend)
        print(f"Plugin overlay: {args.overlay.resolve()}")
        print(f"export AMENT_PREFIX_PATH={args.overlay.resolve()}:$AMENT_PREFIX_PATH")
        return
    package = "control_kit_python" if args.backend == "python" else "control_kit_native"
    if not (args.overlay / f"share/{package}/controllers.xml").is_file():
        parser.error(
            "prepare the plugin overlay with --prepare-only before starting the simulator"
        )

    # Match the source playground instead of inheriting the root Pixi defaults.
    os.environ.update(
        ROS_DOMAIN_ID="77",
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
        ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET",
    )
    import rclpy
    from controller_manager_msgs.srv import (
        ListControllers,
        SwitchController,
        UnloadController,
    )
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from robot_config import ARM_JOINT_GROUPS
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    joints = list(ARM_JOINT_GROUPS[args.side])
    stock = f"arm_{args.side}_controller"
    name = f"cppyy_{args.backend}_arm_{args.side}"
    model = (
        Path(__file__).resolve().parent.parent
        / "tiago_pro_jazzy/generated/robot_description.urdf"
    )
    robot = ET.parse(model).getroot()
    limits = {
        joint.attrib["name"]: joint.find("limit") for joint in robot.findall("joint")
    }
    lower = [float(limits[j].attrib["lower"]) for j in joints]
    upper = [float(limits[j].attrib["upper"]) for j in joints]
    python_evidence = args.report.resolve().with_suffix(".python.json")
    controller_type = ("control_kit_python/PythonController" if args.backend == "python"
                       else "control_kit_native/PositionServo")
    parameters = {"type": controller_type, "joints": joints}
    if args.backend == "python":
        python_evidence.unlink(missing_ok=True)
        python_evidence.parent.mkdir(parents=True, exist_ok=True)
        parameters.update(
            python_module="python_position_controller",
            python_class="PositionServo",
            python_path=str(Path(__file__).resolve().parents[3] / "control_kit/demos"),
            python_options=json.dumps({
                "gain": 6.0, "speed": 0.15, "lower_limits": lower,
                "upper_limits": upper, "evidence_path": str(python_evidence),
            }),
        )
    else:
        parameters.update(gain=6.0, speed_limits=[0.15] * len(joints),
                          lower_limits=lower, upper_limits=upper)
    # JSON is valid YAML and keeps controller parameters unambiguous.
    params = args.overlay / f"{name}.yaml"
    params.write_text(json.dumps({name: {"ros__parameters": parameters}}, indent=2) + "\n")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("cppyy_native_control_proof")
    positions = {}
    received = [0.0]

    def feedback(message):
        positions.update(zip(message.name, message.position))
        received[0] = time.monotonic()

    node.create_subscription(
        JointState, "/joint_states", feedback, qos_profile_sensor_data
    )
    publisher = node.create_publisher(Float64MultiArray, f"/{name}/commands", 1)

    def call(service, service_type, request, timeout=30):
        client = node.create_client(service_type, f"/controller_manager/{service}")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f"No /controller_manager/{service}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                raise TimeoutError(service)
            result = future.result()
            if hasattr(result, "ok") and not result.ok:
                raise RuntimeError(f"{service}: {result}")
            return result
        finally:
            node.destroy_client(client)

    def controllers():
        return {
            c.name: {
                "state": c.state,
                "type": c.type,
                "claimed_interfaces": list(c.claimed_interfaces),
            }
            for c in call(
                "list_controllers", ListControllers, ListControllers.Request()
            ).controller
        }

    def switch(activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        call("switch_controller", SwitchController, request)

    def state():
        if time.monotonic() - received[0] > 2.0:
            raise RuntimeError("Joint feedback is stale")
        return [positions[j] for j in joints]

    def move(target):
        message = Float64MultiArray(data=target)
        samples = []
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            samples.append(state())
        return samples

    plugin_library = args.overlay / f"lib/lib{package}_controller.so"
    report = {
        "plugin_library": str(plugin_library),
        "plugin_sha256": hashlib.sha256(plugin_library.read_bytes()).hexdigest(),
        "controller": controller_type,
        "native_update": args.backend == "native",
        "python_role": ("Control law inside Gazebo; services and observation in the demo"
                        if args.backend == "python" else
                        "ROS services, reference publication and feedback observation"),
        "observer_pid": os.getpid(),
        "joints": joints,
        "offset_rad": args.offset,
        "plugin_overlay": str(args.overlay.resolve()),
    }
    loaded = False
    load_attempted = False
    switched = False
    start = None
    try:
        before = controllers()
        ready_deadline = time.monotonic() + 30
        while (
            before.get(stock, {}).get("state") != "active"
            and time.monotonic() < ready_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
            before = controllers()
        report["controllers_before"] = before
        if before.get(stock, {}).get("state") != "active":
            raise RuntimeError(f"{stock} must be active before this proof")
        if name in before:
            raise RuntimeError(
                f"{name} is already loaded; choose a clean controller manager"
            )
        deadline = time.monotonic() + 20
        while not all(j in positions for j in joints):
            if time.monotonic() > deadline:
                raise TimeoutError("No arm joint feedback")
            rclpy.spin_once(node, timeout_sec=0.1)
        start = state()
        target = start.copy()
        target[-1] += args.offset
        if not lower[-1] + 0.01 < target[-1] < upper[-1] - 0.01:
            raise RuntimeError("Requested move is too close to a joint limit")
        load_attempted = True
        result = subprocess.run(
            [
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                name,
                "-p",
                str(params),
                "--inactive",
                "--controller-manager-timeout",
                "30",
            ],
            text=True,
            capture_output=True,
            timeout=60,
        )
        report["spawner_output"] = result.stdout + result.stderr
        if result.returncode:
            raise RuntimeError(report["spawner_output"])
        loaded = True
        switch([name], [stock])
        switched = True
        report["controllers_during"] = controllers()
        samples = move(target)
        end = samples[-1]
        report.update(
            start_rad=start,
            target_rad=target,
            measured_rad=end,
            sample_count=len(samples),
            measured_displacement_rad=end[-1] - start[-1],
            max_target_error_rad=max(abs(q - t) for q, t in zip(end, target)),
        )
        return_samples = move(start)
        report["returned_rad"] = return_samples[-1]
        report["passed"] = (
            abs(end[-1] - target[-1]) < 0.015
            and abs(end[-1] - start[-1]) > abs(args.offset) * 0.5
            and max(abs(q - t) for q, t in zip(return_samples[-1], start)) < 0.02
        )
        if not report["passed"]:
            raise RuntimeError("Measured arm motion did not meet the proof tolerances")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            if load_attempted:
                current = controllers()
                loaded = name in current
                switched = current.get(name, {}).get("state") == "active"
            if switched:
                switch([stock], [name])
                report["stock_controller_restored"] = True
            if args.backend == "python" and python_evidence.is_file():
                evidence = json.loads(python_evidence.read_text())
                process = Path(f"/proc/{evidence['pid']}")
                evidence["process_command"] = (process / "cmdline").read_bytes().replace(b"\0", b" ").decode()
                evidence["plugin_mapped"] = str(plugin_library) in (process / "maps").read_text()
                report["python_update"] = evidence
            if loaded:
                request = UnloadController.Request(name=name)
                call("unload_controller", UnloadController, request)
            if "controllers_before" in report:
                report["controllers_after"] = controllers()
            if args.backend == "python" and report.get("passed"):
                evidence = report.get("python_update", {})
                if evidence.get("callback_count", 0) <= 0 or not evidence.get("plugin_mapped"):
                    raise RuntimeError("No in-process Python update evidence")
        except BaseException as cleanup_error:
            report["cleanup_error"] = str(cleanup_error)
            raise
        finally:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n")
            node.destroy_node()
            rclpy.shutdown()
    print(
        json.dumps(
            {
                k: report[k]
                for k in (
                    "passed",
                    "measured_displacement_rad",
                    "max_target_error_rad",
                    "stock_controller_restored",
                )
            }
        )
    )


if __name__ == "__main__":
    main()
