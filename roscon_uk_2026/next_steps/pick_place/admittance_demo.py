"""Apply virtual Cartesian forces to one simulated arm, then restore its controller.

Run --prepare-only before launching Gazebo with the printed plugin overlay.
The Python admittance law executes in Gazebo. Its kinematics execute through cppyy.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from native_control_demo import prepare_overlay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay', type=Path, default=Path.home() / '.cache/cppyy_kit/control_plugin')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--duration', type=float, default=8.0, help='Wall seconds for each force and return phase')
    parser.add_argument('--force', type=float, default=1.0, help='Virtual force magnitude, N')
    parser.add_argument('--axis', choices=('x', 'y', 'z'), default='x')
    parser.add_argument(
        '--interactive', action='store_true',
        help='Run browser controls until Ctrl-C or terminal Enter')
    parser.add_argument('--rviz-interactive', action='store_true',
                        help='Use an RViz interactive marker instead of browser controls')
    parser.add_argument('--ui-port', type=int, default=8765)
    parser.add_argument('--report', type=Path, default=Path('admittance_evidence.json'))
    args = parser.parse_args()
    if args.rviz_interactive:
        args.interactive = True
    if not math.isfinite(args.duration) or args.duration < 2:
        parser.error('--duration must be at least 2 seconds')
    if not math.isfinite(args.force) or not 0 < args.force <= 5:
        parser.error('--force must be positive and at most 5 N')
    args.overlay = args.overlay.resolve()
    if args.prepare_only:
        prepare_overlay(args.overlay, 'python')
        print(f'Plugin overlay: {args.overlay}')
        print(f'export AMENT_PREFIX_PATH={args.overlay}:$AMENT_PREFIX_PATH')
        return
    plugin = args.overlay / 'lib/libcontrol_kit_python_controller.so'
    if not plugin.is_file():
        parser.error('run --prepare-only before launching the simulator')
    os.environ.update(ROS_DOMAIN_ID='77', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET')
    import rclpy
    from controller_manager_msgs.srv import ListControllers, SwitchController, UnloadController
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from robot_config import ARM_JOINT_GROUPS
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    demo_path = Path(__file__).resolve().parents[3] / 'control_kit/demos'
    sys.path.insert(0, str(demo_path))
    from arm_kinematics import Chain
    import numpy as np

    joints = list(ARM_JOINT_GROUPS[args.side])
    stock = f'arm_{args.side}_controller'
    name = f'cppyy_admittance_arm_{args.side}'
    model = Path(__file__).resolve().parent.parent / 'tiago_pro_jazzy/generated/robot_description.urdf'
    limits = {j.attrib['name']: j.find('limit') for j in ET.parse(model).getroot().findall('joint')}
    evidence_path = args.report.resolve().with_suffix('.python.json')
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.unlink(missing_ok=True)
    options = {
        'urdf_path': str(model), 'base_link': 'torso_lift_link',
        'tip_link': f'gripper_{args.side}_grasping_link',
        'lower_limits': [float(limits[j].attrib['lower']) for j in joints],
        'upper_limits': [float(limits[j].attrib['upper']) for j in joints],
        'evidence_path': str(evidence_path),
    }
    chain = Chain(str(model), joints, options['base_link'], options['tip_link'])

    def cartesian(q):
        return chain.fk(np.ascontiguousarray(q, dtype=np.float64))[:3, 3].copy()

    params = args.overlay / f'{name}.yaml'
    parameters = {
        'type': 'control_kit_python/PythonController', 'joints': joints,
        'python_module': 'python_admittance_controller',
        'python_class': 'CartesianAdmittance',
        'python_path': str(demo_path), 'python_options': json.dumps(options),
    }
    params.write_text(json.dumps({name: {'ros__parameters': parameters}}, indent=2) + '\n')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('cppyy_admittance_demo')
    positions = {}
    received = [0.0]

    def feedback(message):
        positions.update(zip(message.name, message.position))
        received[0] = time.monotonic()

    node.create_subscription(JointState, '/joint_states', feedback, qos_profile_sensor_data)
    publisher = node.create_publisher(Float64MultiArray, f'/{name}/commands', 1)

    def call(service, service_type, request):
        client = node.create_client(service_type, f'/controller_manager/{service}')
        try:
            if not client.wait_for_service(timeout_sec=30):
                raise TimeoutError(service)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30)
            if not future.done():
                raise TimeoutError(service)
            result = future.result()
            if hasattr(result, 'ok') and not result.ok:
                raise RuntimeError(f'{service}: {result}')
            return result
        finally:
            node.destroy_client(client)

    def controllers():
        return {c.name: {'state': c.state, 'type': c.type,
                         'claimed_interfaces': list(c.claimed_interfaces)}
                for c in call('list_controllers', ListControllers, ListControllers.Request()).controller}

    def switch(activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers, request.deactivate_controllers = activate, deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        call('switch_controller', SwitchController, request)

    def state():
        if time.monotonic() - received[0] > 2:
            raise RuntimeError('Joint feedback is stale')
        return [positions[j] for j in joints]

    def apply(force, duration):
        samples = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            publisher.publish(Float64MultiArray(data=list(force)))
            rclpy.spin_once(node, timeout_sec=0.05)
            samples.append(state())
        return samples

    report = {
        'observer_pid': os.getpid(), 'controller': 'control_kit_python/PythonController',
        'plugin_library': str(plugin), 'joints': joints,
        'force_frame': options['base_link'], 'force_N': args.force,
        'force_axis': args.axis, 'duration_s': args.duration,
        'validation_scope': ('interactive lifecycle' if args.interactive else
                             'automated displacement and spring return'),
    }
    attempted = False
    ui = None
    try:
        before = controllers()
        report['controllers_before'] = before
        if before.get(stock, {}).get('state') != 'active':
            raise RuntimeError(f'{stock} must be active before this demo')
        if name in before:
            raise RuntimeError(f'{name} is already loaded')
        deadline = time.monotonic() + 20
        while not all(j in positions for j in joints):
            if time.monotonic() > deadline:
                raise TimeoutError('No arm feedback')
            rclpy.spin_once(node, timeout_sec=0.1)
        start = state()
        report['start_rad'] = start
        start_xyz = cartesian(start)
        report['start_position_m'] = start_xyz.tolist()
        attempted = True
        result = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', name,
                                 '-p', str(params), '--inactive', '--controller-manager-timeout', '30'],
                                capture_output=True, text=True, timeout=120)
        report['spawner_output'] = result.stdout + result.stderr
        if result.returncode:
            raise RuntimeError(report['spawner_output'])
        switch([name], [stock])
        report['controllers_during'] = controllers()
        if args.interactive:
            import select
            if args.rviz_interactive:
                from admittance_rviz import RvizForce
                ui = RvizForce(node, options['base_link'], start_xyz, max_force=5.0)
                print('RVIZ_FORCE_READY: use Interact, drag the hand marker, release to return.', flush=True)
            else:
                from admittance_ui import start_server
                ui = start_server(port=args.ui_port, frame=options['base_link'], max_force=5.0)
                print(f'Virtual force controls: {ui.url}', flush=True)
            print('Press Enter in this terminal or Ctrl-C to stop.', flush=True)
            report['nonzero_force_samples'] = 0
            try:
                while True:
                    requested_force = ui.get_force()
                    report['nonzero_force_samples'] += int(any(requested_force))
                    sample = apply(requested_force, 0.05)[-1]
                    ui.set_state(position=cartesian(sample).tolist())
                    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                        break
            except KeyboardInterrupt:
                pass
            ui.close()
            ui = None
            report['returned_rad'] = apply((0.0, 0.0, 0.0), args.duration)[-1]
            report['passed'] = True
        else:
            force = [0.0, 0.0, 0.0]
            force['xyz'.index(args.axis)] = args.force
            samples = apply(force, args.duration)
            report['forced_rad'] = samples[-1]
            report['peak_joint_displacement_rad'] = max(abs(q - s) for sample in samples for q, s in zip(sample, start))
            returned = apply((0.0, 0.0, 0.0), args.duration)[-1]
            report['returned_rad'] = returned
            report['forced_position_m'] = cartesian(samples[-1]).tolist()
            report['returned_position_m'] = cartesian(returned).tolist()
            report['peak_cartesian_displacement_m'] = max(
                float(np.linalg.norm(cartesian(q) - start_xyz)) for q in samples)
            report['cartesian_return_error_m'] = float(np.linalg.norm(cartesian(returned) - start_xyz))
            report['max_return_error_rad'] = max(abs(q - s) for q, s in zip(returned, start))
            report['passed'] = (report['peak_cartesian_displacement_m'] > 0.003
                                and report['cartesian_return_error_m'] < 0.008)
            if not report['passed']:
                raise RuntimeError('Arm displacement or spring return did not meet tolerances')
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        try:
            if ui:
                publisher.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
                ui.close()
            if attempted:
                current = controllers()
                if current.get(name, {}).get('state') == 'active':
                    switch([stock], [name])
                    report['stock_controller_restored'] = True
                elif current.get(stock, {}).get('state') == 'inactive':
                    switch([stock], [])
                    report['stock_controller_restored'] = True
                if evidence_path.is_file():
                    evidence = json.loads(evidence_path.read_text())
                    process = Path(f"/proc/{evidence['pid']}")
                    evidence['process_command'] = (process / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
                    evidence['plugin_mapped'] = str(plugin) in (process / 'maps').read_text()
                    report['python_update'] = evidence
                if name in current:
                    call('unload_controller', UnloadController, UnloadController.Request(name=name))
                report['controllers_after'] = controllers()
                if report.get('passed'):
                    after = report['controllers_after']
                    if after.get(stock, {}).get('state') != 'active' or name in after:
                        raise RuntimeError('Stock controller restoration or demo unload failed')
                    evidence = report.get('python_update', {})
                    if not evidence.get('callback_count', 0) or not evidence.get('plugin_mapped'):
                        report['passed'] = False
                        raise RuntimeError('No Gazebo Python callback evidence')
        except BaseException as exc:
            report['passed'] = False
            report['cleanup_error'] = str(exc)
            raise
        finally:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps({k: report[k] for k in ('passed', 'stock_controller_restored')}))


if __name__ == '__main__':
    main()
