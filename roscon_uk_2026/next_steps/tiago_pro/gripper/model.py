"""Restore the URDF mimic relationships using existing bounded physical PID.

Only the follower joints receive forces. The two primary screws remain passive
and keep their existing states; this does not claim commanded gripper actuation.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parent


def configure_primary_effort_control(robot):
    """Add stock effort GripperActionControllers, with one writer per joint."""
    leaders = {f'gripper_{side}_finger_joint' for side in ['left', 'right']}
    seen = set()
    for joint in robot.findall('ros2_control/joint'):
        name = joint.get('name', '')
        if not name.startswith('gripper_'):
            continue
        for interface in list(joint.findall('command_interface')):
            joint.remove(interface)
        if name in leaders:
            ET.SubElement(joint, 'command_interface', name='effort')
            seen.add(name)
        else:
            # External physical PID plugins own follower actuation. Leaving
            # these params would ALSO propagate primary effort in GazeboSystem.
            for param in list(joint.findall('param')):
                if param.get('name') in ['mimic', 'multiplier']:
                    joint.remove(param)
            # Report actual follower positions/velocities for debugging.
            for state in ['position', 'velocity']:
                if joint.find(f"state_interface[@name='{state}']") is None:
                    ET.SubElement(joint, 'state_interface', name=state)
    assert seen == leaders
    configuration = str(ROOT / 'controllers.yaml')
    for plugin in robot.findall('gazebo/plugin'):
        if plugin.get('filename') == 'libgazebo_ros2_control.so':
            if configuration not in [e.text for e in plugin.findall('parameters')]:
                ET.SubElement(plugin, 'parameters').text = configuration
    return ['gripper_left_controller', 'gripper_right_controller']


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def add_physical_mimics(robot):
    """Mutate expanded URDF Element; return a serializable configuration ledger."""
    library = ROOT / 'build/libcppyy_kit_gazebo_mimic.so'
    if not library.is_file():
        raise RuntimeError('Build physical mimic plugin: pixi run bash gripper/build.sh')
    configuration = []
    for joint in robot.findall('joint'):
        mimic = joint.find('mimic')
        if not joint.get('name', '').startswith('gripper_') or mimic is None:
            continue
        child = robot.find(f"link[@name='{joint.find('child').get('link')}']")
        inertial = child.find('inertial')
        mass = float(inertial.find('mass').get('value'))
        if joint.get('type') == 'prismatic':
            effective_inertia = mass
            proportional = 100.0  # N/m, consistent with upstream MuJoCo finger gain.
        else:
            origin = inertial.find('origin')
            xyz = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()])
            rotation = _rotation([float(v) for v in origin.get('rpy', '0 0 0').split()])
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            axis /= np.linalg.norm(axis)
            i = inertial.find('inertia')
            tensor = np.array([[float(i.get('ixx')),float(i.get('ixy')),float(i.get('ixz'))],
                               [float(i.get('ixy')),float(i.get('iyy')),float(i.get('iyz'))],
                               [float(i.get('ixz')),float(i.get('iyz')),float(i.get('izz'))]])
            tensor = rotation @ tensor @ rotation.T + mass*(np.dot(xyz,xyz)*np.eye(3)-np.outer(xyz,xyz))
            effective_inertia = float(axis @ tensor @ axis)
            proportional = 1.0  # Nm/rad; follower compliance, not a measured hardware stiffness.
        # Damping ratio 0.8 from nominal child inertia. Conservative explicit
        # 1ms-step cap avoids excessive derivative feedback for very light links.
        derivative = min(1.6*math.sqrt(proportional*effective_inertia), .8*effective_inertia/.001)
        effort = float(joint.find('limit').get('effort'))
        gazebo = ET.SubElement(robot, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin', name=joint.get('name')+'_physical_mimic', filename=str(library))
        values = {'joint':mimic.get('joint'), 'mimicJoint':joint.get('name'),
                  'multiplier':mimic.get('multiplier','1'), 'offset':mimic.get('offset','0'),
                  'maxEffort':str(effort), 'hasPID':'~/mimic', 'sensitiveness':'0.0'}
        for key, value in values.items():
            ET.SubElement(plugin, key).text = value
        ros = ET.SubElement(plugin, 'ros')
        for key, value in {'p':proportional,'i':0.0,'d':derivative,'i_clamp_min':0.0,'i_clamp_max':0.0}.items():
            ET.SubElement(ros, 'parameter', name='mimic.'+key, type='double').text = str(value)
        ET.SubElement(ros,'parameter',name='mimic.antiwindup',type='bool').text='false'
        configuration.append(dict(name=joint.get('name'), leader=mimic.get('joint'),
                                  multiplier=float(mimic.get('multiplier','1')), proportional=proportional,
                                  derivative=derivative, max_effort=effort,
                                  nominal_child_inertia=effective_inertia))
    assert len(configuration) == 14, f'Expected fourteen followers, got {len(configuration)}'
    return configuration
