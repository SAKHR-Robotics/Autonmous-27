import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    erc_path_planner_dir = get_package_share_directory('erc_path_planner')

    params_file = os.path.join(
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
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
            'params_file': params_file,
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': 'False',
            'autostart': 'true'
        }.items()
    )
    bridge_node = Node(
        package='erc_path_planner',
        executable='costmap_bridge_node',
        name='costmap_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        nav2_launch,
        bridge_node
    ])
