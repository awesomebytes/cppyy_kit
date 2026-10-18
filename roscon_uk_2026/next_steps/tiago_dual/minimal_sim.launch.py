"""Public TIAGo++ model with stock Gazebo and ros2_control components.

This intentionally excludes PAL's optional navigation, joystick and play_motion2
launches so simulation and low-level controls can be checked independently.
"""
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_document = xacro.process_file(
        str(Path(share('tiago_dual_description')) / 'robots/tiago_dual.urdf.xacro'),
        mappings={'use_sim_time': 'true', 'is_public_sim': 'true'},
    )
    # Gazebo defaults sensors to inactive without transport subscribers. The
    # stock ros2_control plugin reads F/T directly, so activate these explicitly.
    for sensor in robot_document.getElementsByTagName('sensor'):
        if sensor.getAttribute('type') == 'force_torque':
            always_on = robot_document.createElement('always_on')
            always_on.appendChild(robot_document.createTextNode('true'))
            sensor.appendChild(always_on)
    # No gripper controller is launched here. Declaring its command interfaces
    # makes GazeboSystem hold those inactive joints using SetVelocity(0), which
    # absorbs external loads outside the wrist constraint feedback. Keep their
    # state interfaces, but leave these deliberately passive fingers to physics.
    for control in robot_document.getElementsByTagName('ros2_control'):
        for joint in control.getElementsByTagName('joint'):
            if joint.getAttribute('name').startswith('gripper_'):
                for interface in list(joint.getElementsByTagName('command_interface')):
                    joint.removeChild(interface)
    robot = robot_document.toxml()
    actions = [
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share('gazebo_ros')) / 'launch/gazebo.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'verbose': 'true'}.items(),
        ),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'tiago_dual', '-z', '0.03'], output='screen'),
    ]
    configs = [
        ('joint_state_broadcaster', 'tiago_dual_controller_configuration', 'joint_state_broadcaster.yaml'),
        ('mobile_base_controller', 'pmb2_controller_configuration', 'mobile_base_controller.yaml'),
        ('torso_controller', 'tiago_dual_controller_configuration', 'torso_controller.yaml'),
        ('head_controller', 'tiago_controller_configuration', 'head_controller.yaml'),
        ('arm_left_controller', 'tiago_dual_controller_configuration', 'arm_left_controller.yaml'),
        ('arm_right_controller', 'tiago_dual_controller_configuration', 'arm_right_controller.yaml'),
        ('ft_sensor_left_controller', 'tiago_dual_controller_configuration', 'ft_sensor_left_controller.yaml'),
        ('ft_sensor_right_controller', 'tiago_dual_controller_configuration', 'ft_sensor_right_controller.yaml'),
    ]
    for name, package, config in configs:
        actions.append(Node(package='controller_manager', executable='spawner',
                            arguments=[name, '--param-file', str(Path(share(package)) / 'config' / config),
                                       '--controller-manager-timeout', '120'], output='screen'))
    return LaunchDescription(actions)
