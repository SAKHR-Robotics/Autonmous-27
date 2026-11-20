"""Compatibility entry point forwarding all documented args to mocked_planner.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mock_share = get_package_share_directory('pathplanner_mock_tools')
    defaults = {
        'map': os.path.join(mock_share, 'custom_maps', 'dummy_map.yaml'),
        'use_rviz': 'false',
        'use_mock_rover': 'true',
        'use_mock_perception': 'true',
        'use_costmap_bridge': 'true',
        'use_external_module': 'false',
        'target_package': 'erc_path_planner',
        'target_launch': 'path_planning.launch.py',
        'cmd_vel_topic': '/cmd_vel',
        'control_mode': 'cmd_vel',
        'initial_x': '0.0',
        'initial_y': '0.0',
        'initial_yaw': '0.0',
        'scenario_id': '',
        'scenarios_file': '',
        'geometry_profile': 'nav2_nominal',
        'seed': '0',
        'clear_default_obstacles': 'false',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mock_share, 'launch', 'mocked_planner.launch.py')
                ),
                launch_arguments={k: LaunchConfiguration(k) for k in defaults}.items(),
            )
        ]
    )
