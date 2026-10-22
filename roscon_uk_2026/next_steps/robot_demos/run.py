"""Start one conference demo and stop only its owned processes on exit."""
import argparse
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

import yaml

HERE = Path(__file__).resolve().parent
SIM = HERE.parent / 'tiago_pro_jazzy'
PICK = HERE.parent / 'pick_place'


def rviz_config(mode):
    config = yaml.safe_load((SIM / 'playground.rviz').read_text())
    view = config['Visualization Manager']
    view['Displays'] = [d for d in view['Displays'] if d['Class'] != 'moveit_rviz_plugin/MotionPlanning']
    view['Displays'].append({
        'Class': 'rviz_default_plugins/RobotModel', 'Name': 'TIAGo Pro',
        'Enabled': True, 'Alpha': 1.0, 'Description Source': 'Topic',
        'Description Topic': {'Value': '/robot_description', 'Depth': 1,
                              'Durability Policy': 'Transient Local', 'Reliability Policy': 'Reliable'},
        'Visual Enabled': True, 'Collision Enabled': False})
    view['Displays'] = [d for d in view['Displays'] if d['Class'] != 'rviz_default_plugins/Image']
    for display in view['Displays']:
        if display.get('Name') in ('Front laser scan', 'Rear laser scan', 'Head RGB-D cloud', 'Global costmap'):
            display['Enabled'] = False
            display['Value'] = False
        if display.get('Name') == 'Local costmap':
            display['Alpha'] = 0.25
    config['Window Geometry'] = {'Width': 1400, 'Height': 950, 'X': 40, 'Y': 40,
                                 'Hide Left Dock': False, 'Hide Right Dock': False}
    orbit = view['Views']['Current']
    if mode == 'admittance':
        view['Displays'] = [d for d in view['Displays'] if d['Class'] in
                            ('rviz_default_plugins/Grid', 'rviz_default_plugins/RobotModel')]
        view['Global Options']['Fixed Frame'] = 'odom'
        view['Displays'] += [
            {'Class': 'rviz_default_plugins/InteractiveMarkers', 'Name': 'Drag to apply virtual force',
             'Enabled': True, 'Interactive Markers Namespace': '/virtual_force',
             'Show Axes': False, 'Show Descriptions': True, 'Show Visual Aids': True},
            {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Force arrow and hand offset',
             'Enabled': True, 'Topic': {'Value': '/virtual_force/markers', 'Depth': 5}}]
        config['Panels'] = [p for p in config['Panels'] if not p['Class'].startswith('nav2_')]
        view['Tools'] = [t for t in view['Tools'] if not t['Class'].startswith('nav2_')]
        orbit.update({'Target Frame': 'torso_lift_link', 'Distance': 1.9,
                      'Focal Point': {'X': .28, 'Y': .25, 'Z': .15}, 'Pitch': .25, 'Yaw': .65})
    else:
        orbit.update({'Target Frame': 'base_footprint', 'Distance': 4.5,
                      'Focal Point': {'X': 0., 'Y': 0., 'Z': .2}, 'Pitch': 1.3, 'Yaw': 0.})
    output = HERE / 'generated' / f'{mode}.rviz'
    output.parent.mkdir(exist_ok=True)
    output.write_text(yaml.safe_dump(config, sort_keys=False))
    return output


def stop(process, timeout):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('baseline', 'forward', 'admittance'))
    parser.add_argument('--headless', action='store_true', help='Validation without Gazebo/RViz windows')
    parser.add_argument('--gazebo-gui', action='store_true', help='Also open the Gazebo rendering window')
    parser.add_argument('--scene', choices=('house', 'simple-house', 'table'), default='simple-house')
    parser.add_argument('--run-seconds', type=float, help='Stop after this many seconds, for recording/validation')
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID='77', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET', GZ_PARTITION='tiago_pro_jazzy',
                      TIAGO_PRO_NAV_PROFILE='forward' if args.mode == 'forward' else 'default',
                      TIAGO_PRO_NAV_DEPTH='1', PYTHONUNBUFFERED='1',
                      TIAGO_PRO_SCENE=args.scene)
    overlay = Path.home() / '.cache/cppyy_kit/control_plugin'
    if args.mode == 'admittance':
        subprocess.run([sys.executable, str(PICK/'admittance_demo.py'), '--prepare-only'], check=True)
        os.environ['AMENT_PREFIX_PATH'] = str(overlay) + ':' + os.environ.get('AMENT_PREFIX_PATH', '')
    config = rviz_config(args.mode)
    command = ['ros2', 'launch', str(SIM/'playground.launch.py'),
               'gui:=' + str(args.gazebo_gui and not args.headless).lower(),
               'rviz:=' + str(not args.headless).lower(),
               'moveit:=false', 'navigation:=' + str(args.mode != 'admittance').lower(),
               'rviz_config:=' + str(config)]
    if args.scene == 'simple-house' and args.mode != 'admittance':
        map_dir = HERE / 'generated/simple-house'
        subprocess.run([sys.executable, str(SIM.parent / 'tiago_pro/worlds/home/prepare.py'),
                        '--scene', 'simple-house', '--output', str(map_dir)], check=True)
        command.append('map:=' + str(map_dir / 'map.yaml'))
    if args.scene == 'table' and args.mode != 'admittance':
        # The table-only scene has no house walls. Live sensors supply obstacles.
        map_dir = HERE / 'generated'
        (map_dir / 'table.pgm').write_bytes(b'P5\n400 400\n255\n' + bytes([254]) * 160000)
        map_file = map_dir / 'table.yaml'
        map_file.write_text(yaml.safe_dump({
            'image': 'table.pgm', 'resolution': 0.05, 'origin': [-9., -7., 0.],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}))
        command.append('map:=' + str(map_file))
    evidence = HERE / 'evidence'
    evidence.mkdir(exist_ok=True)
    messages = queue.Queue()
    children = []
    logs = []

    def spawn(label, cmd):
        logfile = (evidence / f'{args.mode}-{label}.log').open('w')
        logs.append(logfile)
        process = subprocess.Popen(cmd, cwd=SIM, start_new_session=True,
                                   stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        children.append(process)

        def reader():
            for line in process.stdout:
                logfile.write(line)
                logfile.flush()
                messages.put((label, line))
        threading.Thread(target=reader, daemon=True).start()
        return process

    launch = controller = None
    started = time.monotonic()
    try:
        launch = spawn('sim', command)
        print(f'Demo: {args.mode}. Ctrl-C stops this stack. Logs: {evidence}', flush=True)
        while launch.poll() is None:
            if args.run_seconds and time.monotonic() - started >= args.run_seconds:
                break
            try:
                label, line = messages.get(timeout=.2)
                print(line, end='', flush=True)
                if label == 'sim' and 'TIAGO_PRO_READY:' in line:
                    if args.mode == 'admittance' and controller is None:
                        controller = spawn('controller', [sys.executable, str(PICK/'admittance_demo.py'),
                            '--rviz-interactive', '--report', str(evidence/'admittance.json')])
                    elif args.mode != 'admittance':
                        print('NAV_DEMO: wait for Navigation 2 to become active; send an RViz goal.', flush=True)
            except queue.Empty:
                pass
            if controller is not None and controller.poll() is not None:
                if controller.returncode:
                    raise RuntimeError(f'Admittance process exited {controller.returncode}; see its log')
                break
        if launch.poll() not in (None, 0):
            raise RuntimeError(f'Simulator launch exited {launch.returncode}')
    except KeyboardInterrupt:
        pass
    finally:
        # Restore arm controller while Gazebo and its services are still alive.
        stop(controller, 35)
        stop(launch, 25)
        for logfile in logs:
            logfile.flush()
    print('Demo stopped.', flush=True)


if __name__ == '__main__':
    main()
