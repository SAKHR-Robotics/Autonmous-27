"""Compatibility entry point; combined startup belongs to perception_bringup."""
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    try:
        share = get_package_share_directory('perception_bringup')
    except PackageNotFoundError as error:
        raise RuntimeError(
            'Build perception_bringup for combined startup. For terrain only, use '
            'ros2 launch terrain_geometry terrain.launch.py') from error
    return LaunchDescription([
        LogInfo(msg='Deprecated entry point: use ros2 launch perception_bringup perception.launch.py'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'perception.launch.py'))),
    ])
