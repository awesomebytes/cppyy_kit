#!/usr/bin/env python
"""Run the installed playground from a small writable session directory."""
import os
from pathlib import Path
import shutil
import sys
import tempfile

prefix = Path(sys.executable).resolve().parent.parent
resources = prefix / 'share/tiago_pro_playground'
args = sys.argv[1:]
if '--help' in args:
    print('tiago-pro-playground [ROS launch arguments, e.g. gui:=false rviz:=false]\n'
          'Uses the bundled home world. --prepare-only generates model/configuration without starting ROS.\n'
          'RViz layout: $XDG_CONFIG_HOME/tiago-pro-playground/playground.rviz\n'
          'Sessions: $XDG_CACHE_HOME/tiago-pro-playground/')
    raise SystemExit(0)
cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'tiago-pro-playground'
cache.mkdir(parents=True, exist_ok=True)
session = Path(tempfile.mkdtemp(prefix='session-', dir=cache))
shutil.copytree(resources / 'runtime', session, dirs_exist_ok=True)
(session / 'worlds').symlink_to(resources / 'worlds', target_is_directory=True)
(session / 'generated').mkdir()
(session / 'generated/prefix').symlink_to(prefix, target_is_directory=True)
for local, package in [('upstream', 'tiago_pro_moveit_config'),
                       ('pal_sea_arm_moveit_config', 'pal_sea_arm_moveit_config')]:
    (session / 'moveit' / local).symlink_to(prefix / 'share' / package, target_is_directory=True)
for local, library in [('build/planar_plugin', 'libcppyy_kit_gazebo_ros_planar_move.so'),
                       ('gripper/build', 'libcppyy_kit_gazebo_mimic.so')]:
    directory = session / local
    directory.mkdir(parents=True, exist_ok=True)
    (directory / library).symlink_to(prefix / 'lib' / library)
config = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'tiago-pro-playground'
config.mkdir(parents=True, exist_ok=True)
rviz = config / 'playground.rviz'
try:
    with rviz.open('xb') as output:
        output.write((session / 'playground.rviz').read_bytes())
except FileExistsError:
    pass
os.environ['TIAGO_PRO_INSTALLED_PREFIX'] = str(prefix)
os.environ['AMENT_PREFIX_PATH'] = str(prefix) + ':' + os.environ.get('AMENT_PREFIX_PATH', '')
os.environ['ROS_LOG_DIR'] = str(session / 'logs')
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
os.chdir(session)
print(f'TIAGo PRO session: {session}', flush=True)
if '--prepare-only' in args:
    import ctypes
    for library in ['libcppyy_kit_gazebo_ros_planar_move.so', 'libcppyy_kit_gazebo_mimic.so']:
        ctypes.CDLL(str(prefix / 'lib' / library))
    sys.path[:0] = [str(session), str(session / 'moveit')]
    from simulation_model import prepare
    from configure import configure
    prepare()
    configure()
    print('Installed model, patched plugins and MoveIt configuration resolved successfully.', flush=True)
else:
    if not any(arg.startswith('rviz_config:=') for arg in args):
        args.append(f'rviz_config:={rviz}')
    os.execvp('bash', ['bash', str(resources / 'simulator_guard.sh'), '--',
                      'ros2', 'launch', str(session / 'playground.launch.py'), *args])
