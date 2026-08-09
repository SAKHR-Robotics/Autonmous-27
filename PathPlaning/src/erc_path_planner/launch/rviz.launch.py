from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('erc_path_planner'),
        'rviz',
        'nav2_default_view.rviz'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])