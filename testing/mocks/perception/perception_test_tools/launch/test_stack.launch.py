import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    pipeline = LaunchConfiguration('pipeline').perform(context)
    if LaunchConfiguration('launch_memory').perform(context) == 'true' and pipeline != 'terrain':
        raise ValueError('Synthetic motion is a terrain-memory fixture; select pipeline:=terrain')

    def include(package, filename, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), 'launch', filename)),
            launch_arguments=arguments.items())])

    return [
        include('perception_test_tools', 'sensor_inputs.launch.py', {
            'scenario': LaunchConfiguration('scenario'),
            'synthetic_motion': LaunchConfiguration('launch_memory'),
            'use_sim_time': 'false',
            'ground_z_offset': LaunchConfiguration('ground_z_offset'),
            'seed': LaunchConfiguration('seed'),
            'rate': LaunchConfiguration('rate'),
        }),
        include('terrain_geometry', 'perception_system.launch.py', {
            'use_sim_time': 'false',
            'launch_terrain': 'false' if pipeline == 'markers' else 'true',
            'launch_marker_branch': 'false' if pipeline == 'terrain' else 'true',
            'launch_marker_mapping': LaunchConfiguration('launch_marker_mapping'),
            'launch_landmark_adapter': LaunchConfiguration('launch_landmark_adapter'),
            'launch_memory': LaunchConfiguration('launch_memory'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_config': os.path.join(get_package_share_directory('perception_test_tools'), 'rviz', 'perception_test_view.rviz'),
            'enable_costmap': 'true',
            'input_pointcloud_topic': '/testing/perception/camera/points',
            'rgb_topic': '/testing/perception/camera/image_raw',
            'depth_topic': '/testing/perception/camera/depth',
            'camera_info_topic': '/testing/perception/camera/image_info',
        }),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pipeline', default_value='all', choices=['all', 'terrain', 'markers']),
        DeclareLaunchArgument('scenario', default_value='rocks_and_marker'),
        DeclareLaunchArgument('launch_memory', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('ground_z_offset', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('rate', default_value='10.0'),
        DeclareLaunchArgument('launch_landmark_adapter', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_marker_mapping', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
