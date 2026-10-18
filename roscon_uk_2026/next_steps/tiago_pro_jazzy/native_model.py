"""Native Harmonic base systems and bounded-force gripper followers."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _plugin(robot, filename, name, values):
    plugin = ET.SubElement(ET.SubElement(robot, 'gazebo'), 'plugin',
                           filename=filename, name=name)
    for key, value in values.items():
        ET.SubElement(plugin, key).text = str(value)
    return plugin


def configure(robot):
    """Attach native systems and return ros_gz_bridge configuration entries."""
    library = ROOT / 'native/build/libtiago_pro_physical_mimic.so'
    if not library.is_file():
        raise RuntimeError('Build native follower plugin with pixi run bash build_native.sh')
    _plugin(robot, 'gz-sim-velocity-control-system',
            'gz::sim::systems::VelocityControl', {'topic': '/tiago_pro/cmd_vel'})
    _plugin(robot, 'gz-sim-odometry-publisher-system',
            'gz::sim::systems::OdometryPublisher', {
                'odom_frame': 'odom', 'robot_base_frame': 'base_footprint',
                'dimensions': 2, 'odom_publish_frequency': 50,
                'odom_topic': '/tiago_pro/odom', 'tf_topic': '/tiago_pro/odom_tf'})
    physical = _plugin(robot, str(library), 'tiago_pro::PhysicalMimic', {})
    count = 0
    for joint in robot.findall('joint'):
        mimic = joint.find('mimic')
        if not joint.get('name', '').startswith('gripper_') or mimic is None:
            continue
        name = joint.get('name')
        controls = robot.findall(f"ros2_control/joint[@name='{name}']")
        for control in controls:
            # Jazzy otherwise creates velocity servos for URDF mimics.
            control.set('mimic', 'false')
            for interface in list(control.findall('command_interface')):
                control.remove(interface)
            for param in list(control.findall('param')):
                if param.get('name') in ('mimic', 'multiplier', 'offset'):
                    control.remove(param)
            for state in ('position', 'velocity'):
                if control.find(f"state_interface[@name='{state}']") is None:
                    ET.SubElement(control, 'state_interface', name=state)
        child = robot.find(f"link[@name='{joint.find('child').get('link')}']")
        inertial = child.find('inertial')
        mass = float(inertial.find('mass').get('value'))
        if joint.get('type') == 'prismatic':
            effective_inertia, proportional = mass, 100.0
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
            effective_inertia, proportional = float(axis @ tensor @ axis), 1.0
        derivative = min(1.6*math.sqrt(proportional*effective_inertia), .8*effective_inertia/.001)
        follower = ET.SubElement(physical, 'follower')
        for key, value in {'leader': mimic.get('joint'), 'joint': name,
                           'multiplier': mimic.get('multiplier', '1'),
                           'offset': mimic.get('offset', '0'),
                           'p': proportional, 'd': derivative,
                           'max_effort': joint.find('limit').get('effort')}.items():
            ET.SubElement(follower, key).text = str(value)
        count += 1
    if count != 14:
        raise ValueError(f'Expected fourteen physical gripper followers, got {count}')
    return [
        dict(ros_topic_name='/mobile_base_controller/cmd_vel_unstamped', gz_topic_name='/tiago_pro/cmd_vel',
             ros_type_name='geometry_msgs/msg/Twist', gz_type_name='gz.msgs.Twist',
             direction='ROS_TO_GZ'),
        dict(ros_topic_name='/mobile_base_controller/odom', gz_topic_name='/tiago_pro/odom',
             ros_type_name='nav_msgs/msg/Odometry', gz_type_name='gz.msgs.Odometry',
             direction='GZ_TO_ROS'),
        dict(ros_topic_name='/tf', gz_topic_name='/tiago_pro/odom_tf',
             ros_type_name='tf2_msgs/msg/TFMessage', gz_type_name='gz.msgs.Pose_V',
             direction='GZ_TO_ROS'),
    ]
