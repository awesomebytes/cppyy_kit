"""Stock Nav2 for the empty PRO world, with explicit simulation truth localization."""
from pathlib import Path
import copy
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent
    artifacts = root / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    # Match the obstacle-free simulator plane within a 20 m working region.
    (artifacts / 'empty.pgm').write_bytes(b'P5\n200 200\n255\n' + bytes([254]) * 40000)
    (artifacts / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'empty.pgm', 'resolution': 0.1, 'origin': [-10.0, -10.0, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}))
    stock = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
    params = yaml.safe_load(stock.read_text())
    for value in params.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = True
    for name in ['bt_navigator', 'behavior_server']:
        params[name]['ros__parameters']['robot_base_frame'] = 'base_footprint'
        params[name]['ros__parameters']['enable_stamped_cmd_vel'] = False
    params['bt_navigator']['ros__parameters']['odom_topic'] = '/mobile_base_controller/odom'
    controller = params['controller_server']['ros__parameters']
    controller.update(enable_stamped_cmd_vel=False, controller_frequency=15.0, min_y_velocity_threshold=0.001,
                      odom_topic='/mobile_base_controller/odom')
    controller['progress_checker'].update(required_movement_radius=0.1, movement_time_allowance=20.0)
    controller['general_goal_checker'].update(xy_goal_tolerance=0.1, yaw_goal_tolerance=0.12)
    controller['FollowPath'] = yaml.safe_load((root / 'dwb.yaml').read_text())
    controller['FollowPath'].update(
        min_vel_x=-0.2, min_vel_y=-0.2, max_vel_x=0.25, max_vel_y=0.25,
        max_speed_xy=0.25, max_vel_theta=0.5, acc_lim_x=0.5, acc_lim_y=0.5,
        decel_lim_x=-0.5, decel_lim_y=-0.5, acc_lim_theta=1.0, decel_lim_theta=-1.0,
        vx_samples=12, vy_samples=9, vtheta_samples=16, trans_stopped_velocity=0.03,
        xy_goal_tolerance=0.1)
    observations = {
        name: {'topic': topic, 'max_obstacle_height': 2.0, 'clearing': True,
               'marking': True, 'data_type': 'LaserScan', 'inf_is_valid': True,
               'raytrace_max_range': 8.0, 'obstacle_max_range': 7.5,
               'raytrace_min_range': 0.15, 'obstacle_min_range': 0.15}
        for name, topic in [('front', '/scan_front_raw'), ('rear', '/scan_rear_raw')]}
    for name in ['local_costmap', 'global_costmap']:
        costmap = params[name][name]['ros__parameters']
        costmap.update(robot_base_frame='base_footprint', use_sim_time=True,
                       # Full ready-pose collision projection: x[-.397,.383], y[-.308,.308].
                       # At least 3 cm margin before the explicit 1 cm costmap padding.
                       footprint='[[0.43,0.34],[0.43,-0.34],[-0.43,-0.34],[-0.43,0.34]]',
                       footprint_padding=0.01,
                       transform_tolerance=0.5, always_send_full_costmap=True)
        costmap.pop('robot_radius', None)
        costmap['plugins'] = (['static_layer'] if name == 'global_costmap' else []) + ['obstacle_layer', 'inflation_layer']
        costmap['obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
            'observation_sources': 'front rear', **copy.deepcopy(observations)}
        costmap['inflation_layer'].update(inflation_radius=0.65, cost_scaling_factor=3.0)
    selected_map = Path(os.environ.get('TIAGO_PRO_NAV_MAP', str(artifacts / 'map.yaml'))).resolve()
    if not selected_map.is_file():
        raise FileNotFoundError(selected_map)
    params.setdefault('map_server', {'ros__parameters': {'use_sim_time': True}})['ros__parameters']['yaml_filename'] = str(selected_map)
    params_file = artifacts / 'nav2_params.yaml'
    params_file.write_text(yaml.safe_dump(params))
    nodes = [('nav2_map_server', 'map_server', 'map_server'),
             ('nav2_controller', 'controller_server', 'controller_server'),
             ('nav2_planner', 'planner_server', 'planner_server'),
             ('nav2_smoother', 'smoother_server', 'smoother_server'),
             ('nav2_behaviors', 'behavior_server', 'behavior_server'),
             ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator')]
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='simulation_truth_map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),
        *[Node(package=package, executable=executable, name=name,
               parameters=[str(params_file)], output='screen',
               remappings=[('cmd_vel', '/mobile_base_controller/cmd_vel_unstamped')])
          for package, executable, name in nodes],
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': [name for _, _, name in nodes]}]),
    ])
