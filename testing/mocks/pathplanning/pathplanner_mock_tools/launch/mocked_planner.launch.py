"""Run real production planning/control with opt-in synthetic dependencies."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def setup(context):
    mock_share = get_package_share_directory('pathplanner_mock_tools')
    production = get_package_share_directory('erc_path_planner')
    # No simulated camera is supplied by this profile: marking only, no invented
    # sensor-origin clearing rays. Obstacle removal/clearing needs real sensors.
    params = RewrittenYaml(source_file=os.path.join(production, 'config', 'nav2_params.yaml'),
                          param_rewrites={'observation_sources': 'obstacles'}, convert_types=True)
    target = os.path.join(production, 'launch', 'path_planning.launch.py')
    if LaunchConfiguration('use_external_module').perform(context).lower() == 'false':
        target = os.path.join(get_package_share_directory(LaunchConfiguration('target_package').perform(context)),
                              'launch', LaunchConfiguration('target_launch').perform(context))
    return [
        Node(package='rviz2', executable='rviz2', name='pathplanner_test_rviz',
             arguments=['-d', os.path.join(mock_share, 'rviz', 'mocked_planner.rviz')],
             parameters=[{'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target),
            condition=UnlessCondition(LaunchConfiguration('use_external_module')),
            launch_arguments={'use_slam': 'false', 'map': LaunchConfiguration('map'),
                              'params_file': params, 'use_sim_time': 'false',
                              'launch_bridge': LaunchConfiguration('use_costmap_bridge'),
                              'launch_rviz': 'false'}.items()),
        Node(package='pathplanner_mock_tools', executable='mock_rover_sim', name='mock_rover_sim',
             output='screen', condition=IfCondition(LaunchConfiguration('use_mock_rover')),
             parameters=[{'use_sim_time': False, 'publish_tf': True,
                 'initial_x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                 'initial_y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
                 'initial_yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                 'control_mode': LaunchConfiguration('control_mode'),
                 'rover_length': 0.9, 'rover_width': 0.7,
                 'slip_ratio_linear': 0.0, 'slip_ratio_angular': 0.0,
                 'noise_std_v': 0.0, 'noise_std_w': 0.0}]),
        Node(package='pathplanner_mock_tools', executable='mock_perception', name='mock_perception',
             output='screen', condition=IfCondition(LaunchConfiguration('use_mock_perception')),
             parameters=[{'use_sim_time': False, 'publish_detections': True,
                 'publish_bridge_cloud': False, 'scenario_id': LaunchConfiguration('scenario_id'),
                 'scenarios_file': LaunchConfiguration('scenarios_file')}])]


def generate_launch_description():
    share = get_package_share_directory('pathplanner_mock_tools')
    defaults = {'map': os.path.join(share, 'custom_maps', 'dummy_map.yaml'),
        'use_rviz': 'false', 'use_mock_rover': 'true', 'use_mock_perception': 'true',
        'use_costmap_bridge': 'true', 'use_external_module': 'false',
        'target_package': 'erc_path_planner', 'target_launch': 'path_planning.launch.py',
        'cmd_vel_topic': '/cmd_vel', 'control_mode': 'cmd_vel',
        'initial_x': '0.0', 'initial_y': '0.0', 'initial_yaw': '0.0',
        'scenario_id': '', 'scenarios_file': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
