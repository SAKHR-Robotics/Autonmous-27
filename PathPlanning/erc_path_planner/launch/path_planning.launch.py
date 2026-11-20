"""Production-only composition. Map, localization, sensors and TF are external."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share = get_package_share_directory('erc_path_planner')
    actions = []
    lifecycle_nodes = []
    # Saved real maps are supported, but no synthetic/default map is installed here.
    if LaunchConfiguration('use_slam').perform(context).lower() not in ('true', '1'):
        map_file = LaunchConfiguration('map').perform(context)
        if not os.path.isabs(map_file) or not os.path.isfile(map_file):
            raise ValueError('use_slam:=false requires map:=/absolute/path/to/a/map.yaml')
        clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
        actions += [
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[{'yaml_filename': map_file, 'use_sim_time': clock}])]
        lifecycle_nodes.append('map_server')
    common = {key: LaunchConfiguration(key) for key in
              ('use_sim_time', 'autostart', 'params_file', 'odom_topic')}
    common['manage_lifecycle'] = 'false'
    owned = {'planner': ['planner_server'], 'controller': ['controller_server'],
             'navigation': ['behavior_server', 'velocity_smoother', 'bt_navigator']}
    for part in ('planner', 'controller', 'navigation'):
        if LaunchConfiguration('launch_' + part).perform(context).lower() in ('true', '1'):
            lifecycle_nodes.extend(owned[part])
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', part + '.launch.py')),
            launch_arguments=common.items(), condition=IfCondition(LaunchConfiguration('launch_' + part))))
    actions += [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'obstacle_bridge.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'input_topic': LaunchConfiguration('obstacles_topic')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_bridge'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rviz.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_rviz')))]
    if lifecycle_nodes:
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_pathplanning', output='screen', parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                'node_names': lifecycle_nodes, 'bond_timeout': 4.0}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('erc_path_planner')
    defaults = {
        'use_slam': 'true', 'map': '', 'use_sim_time': 'false', 'autostart': 'true',
        'params_file': os.path.join(share, 'config', 'nav2_params.yaml'),
        'odom_topic': '/odometry/filtered', 'obstacles_topic': '/perception/obstacles_only',
        'launch_planner': 'true', 'launch_controller': 'true', 'launch_navigation': 'true',
        'launch_bridge': 'true', 'launch_rviz': 'false'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v,
        choices=['true', 'false'] if k.startswith('launch_') or k in ('use_slam', 'use_sim_time', 'autostart') else None)
        for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
