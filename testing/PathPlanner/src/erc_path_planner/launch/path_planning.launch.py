from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    erc_path_planner_dir = get_package_share_directory('erc_path_planner')

    default_params_file = os.path.join(
        erc_path_planner_dir,
        'config',
        'nav2_params.yaml'
    )

    default_map = os.path.join(
        erc_path_planner_dir,
        'config',
        'dummy_map.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    # NEW: allow overriding which params file (and therefore which global
    # planner plugin, e.g. SmacPlannerHybrid vs SmacPlanner2D/A*) is used,
    # without needing to edit this launch file each time.
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the nav2 params yaml file to load'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        nav2_launch
    ])
