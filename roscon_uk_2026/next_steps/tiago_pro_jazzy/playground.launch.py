"""One ROS launch description for the PRO simulator and optional robot stacks."""
import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

ROOT = Path(__file__).resolve().parent
ASSETS = Path(os.environ.get("TIAGO_PRO_RESOURCE_ROOT", ROOT.parent / "tiago_pro"))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'moveit'))


def enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def setup(context):
    from configure import configure
    from validate_rviz import validate

    world = Path(LaunchConfiguration('world').perform(context)).resolve()
    map_file = Path(LaunchConfiguration('map').perform(context)).resolve()
    rviz_file = Path(LaunchConfiguration('rviz_config').perform(context)).resolve()
    os.environ['ROS_DOMAIN_ID'] = '77'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['GZ_PARTITION'] = 'tiago_pro_jazzy'
    os.environ['TIAGO_PRO_NAV_MAP'] = str(map_file)
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join([str(ROOT / 'generated/prefix/share'), str(ASSETS / 'worlds/home/models'), *[str(p) for p in (ASSETS / 'worlds/home/models').glob('*/materials/textures')]])
    os.environ.setdefault('QT_X11_NO_MITSHM', '1')

    if enabled(context, 'simulation'):
        from simulation_model import prepare
        description, grippers = prepare()
    else:
        description = (ROOT / 'generated/robot_description.urdf').read_text()
        grippers = []
    parameters = configure() if enabled(context, 'moveit') or enabled(context, 'rviz') else None
    environment = [SetEnvironmentVariable(key, os.environ[key]) for key in [
        'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'GZ_PARTITION', 'TIAGO_PRO_NAV_MAP',
        'GZ_SIM_RESOURCE_PATH', 'QT_X11_NO_MITSHM', 'AMENT_PREFIX_PATH']]

    rest = []
    if enabled(context, 'moveit'):
        move_group = Node(package='moveit_ros_move_group', executable='move_group',
                          parameters=[str(parameters)], output='screen')
        rest.extend([move_group, RegisterEventHandler(OnProcessExit(
            target_action=move_group,
            on_exit=[EmitEvent(event=Shutdown(reason='MoveIt exited'))]))])
    if enabled(context, 'navigation'):
        rest.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(ROOT / 'navigation/nav.launch.py'))))
    if enabled(context, 'rviz'):
        validate(rviz_file)
        rest.append(Node(package='rviz2', executable='rviz2',
                         arguments=['-d', str(rviz_file)], parameters=[str(parameters)],
                         output='screen'))
        # RViz is optional: closing it leaves the simulator and robot servers alive.
    if not enabled(context, 'simulation'):
        return environment + rest

    server = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', str(ROOT / 'generated/home.sdf')], output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': str(ROOT / 'generated/bridge.yaml')}], output='screen')
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'use_sim_time': True, 'robot_description': description}],
                     output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'tiago_pro', '-file', str(ROOT / 'generated/sim.urdf'),
                            '-z', '0.02', '-x', LaunchConfiguration('spawn_x'),
                            '-y', LaunchConfiguration('spawn_y'), '-Y', LaunchConfiguration('spawn_yaw')])
    controllers = ['joint_state_broadcaster', 'arm_left_controller', 'arm_right_controller',
                   'torso_controller', 'head_controller', 'wrist_left_ft_broadcaster',
                   'wrist_right_ft_broadcaster'] + grippers
    steps = [spawn] + [Node(package='controller_manager', executable='spawner',
                           arguments=[name, '--controller-manager-timeout', '60', '--switch-timeout', '30'],
                           output='screen') for name in controllers]
    if enabled(context, 'gui'):
        rest.insert(0, ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'))
    rest.insert(0, LogInfo(msg='TIAGO_PRO_READY: simulator and native controllers active'))

    def next_step(next_actions):
        def on_exit(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Robot startup process failed'))]
            return next_actions
        return on_exit

    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=step, on_exit=next_step([steps[index + 1]] if index + 1 < len(steps) else rest)))
        for index, step in enumerate(steps)]
    return environment + handlers + [
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        server, publisher, bridge, spawn]


def generate_launch_description():
    defaults = {
        'simulation': 'true', 'moveit': 'true', 'navigation': 'true',
        'rviz': 'true', 'gui': 'true',
        'world': str(ASSETS / 'worlds/home/home.world'),
        'map': str(ASSETS / 'worlds/home/map.yaml'),
        'spawn_x': '-2.65728', 'spawn_y': '1.35', 'spawn_yaw': '1.5707963267948966',
        'rviz_config': str(ROOT / 'playground.rviz'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=setup),
    ])
