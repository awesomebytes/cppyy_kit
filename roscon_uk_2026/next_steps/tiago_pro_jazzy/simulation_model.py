"""Prepare the pinned PAL descriptions for Jazzy and Gazebo Harmonic."""
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from initial_pose import configure as configure_initial_pose

ROOT = Path(__file__).resolve().parent
ASSETS = Path(os.environ.get('TIAGO_PRO_RESOURCE_ROOT', ROOT.parent / 'tiago_pro'))


def prepare():
    import prepare_model
    robot = ET.fromstring((ROOT / 'generated/tiago_pro.urdf').read_text())
    plugin = robot.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    plugin.find('parameters').text = str(ROOT / 'controllers.yaml')
    for side in ['left', 'right']:
        sensing = robot.find(f"joint[@name='wrist_{side}_ft_sensor_joint']")
        tool = robot.find(f"joint[@name='wrist_{side}_ft_tool_joint']")
        a = [float(v) for v in sensing.find('origin').get('xyz').split()]
        b = [float(v) for v in tool.find('origin').get('xyz').split()]
        tool.find('parent').set('link', f'wrist_{side}_ft_sensor_link')
        tool.find('origin').set('xyz', ' '.join(str(y-x) for x,y in zip(a,b)))
    # Reuse RGBD's aligned color output instead of rendering a second RGB camera.
    for gazebo in robot.findall('gazebo'):
        for sensor in list(gazebo.findall("sensor[@type='camera']")):
            if sensor.get('name') == 'head_front_camera':
                gazebo.remove(sensor)
    for sensor in robot.findall(".//sensor[@type='rgbd_camera']"):
        ET.SubElement(sensor.find('camera'), 'optical_frame_id').text = 'head_front_camera_depth_optical_frame'
    for image_format in robot.findall('.//sensor/camera/image/format'):
        image_format.text = 'R8G8B8'
    for sensor in robot.findall('.//sensor'):
        if sensor.get('type') == 'force_torque':
            ET.SubElement(sensor, 'always_on').text = 'true'
    leaders = {f'gripper_{s}_finger_joint' for s in ['left','right']}
    for joint in robot.findall('ros2_control/joint'):
        if joint.get('name','').startswith('gripper_'):
            for interface in list(joint.findall('command_interface')):
                joint.remove(interface)
            if joint.get('name') in leaders:
                ET.SubElement(joint, 'command_interface', name='effort')
    from native_model import configure
    native_bridges = configure(robot)
    configure_initial_pose(robot)
    description = ET.tostring(robot, encoding='unicode')
    (ROOT / 'generated/robot_description.urdf').write_text(description)
    for joint in robot.findall('joint'):
        mimic = joint.find('mimic')
        if joint.get('name','').startswith('gripper_') and mimic is not None:
            joint.remove(mimic)
    sim = re.sub(r'package://([^/]+)/',lambda m: get_package_share_directory(m[1])+'/',ET.tostring(robot, encoding='unicode'))
    (ROOT / 'generated/sim.urdf').write_text(sim)
    world = ET.parse(ASSETS / 'worlds/home/home.world')
    w = world.getroot().find('world')
    w.find('physics').set('type','ignored')
    for filename, name in [('physics','Physics'),('user-commands','UserCommands'),
                           ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),
                           ('imu','Imu'),('forcetorque','ForceTorque')]:
        p = ET.SubElement(w,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{name}')
        if name == 'Sensors': ET.SubElement(p,'render_engine').text='ogre2'
    from world_materials import prepare as prepare_materials
    prepare_materials(world, ASSETS / 'worlds/home/models', ROOT / 'generated')
    world.write(ROOT / 'generated/home.sdf',encoding='unicode')
    bridge = yaml.safe_load((ROOT / 'bridge.yaml').read_text())
    bridge = [b for b in bridge if b['ros_type_name'] != 'nav_msgs/msg/Odometry'] + (native_bridges or [])
    (ROOT / 'generated/bridge.yaml').write_text(yaml.safe_dump(bridge))
    return description, ['gripper_left_controller','gripper_right_controller']
