"""Minimal upstream TIAGo Classic simulation; optional stacks remain separate."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = lambda package: Path(get_package_share_directory(package))
    robot = xacro.process_file(str(share('tiago_description') / 'robots/tiago.urdf.xacro'), mappings={
        'use_sim_time': 'true', 'is_public_sim': 'true', 'gazebo_version': 'classic',
        'end_effector': 'pal-gripper', 'ft_sensor': 'no-ft-sensor',
    }).toxml()
    packages = ['tiago_description','pmb2_description','pal_hey5_description','pal_gripper_description','pal_robotiq_description','omni_base_description','pal_urdf_utils']
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(str(share(p).parent) for p in packages)
    os.environ['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:11361'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', str(Path(__file__).parent/'playground.world')], output='screen'),
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'use_sim_time':True,'robot_description':robot}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic','robot_description','-entity','tiago','-z','0.03'], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('tiago_controller_configuration')/'launch/default_controllers.launch.py')), launch_arguments={'use_sim_time':'True','is_public_sim':'True','arm_type':'tiago-arm','end_effector':'pal-gripper','ft_sensor':'no-ft-sensor','base_type':'pmb2'}.items()),
    ])
