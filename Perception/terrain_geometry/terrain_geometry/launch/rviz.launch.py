"""Open the terrain view without starting processing or sensor nodes."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("terrain_geometry"), "rviz", "terrain_view.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fixed_frame", default_value="base_link"),
        DeclareLaunchArgument("input_pointcloud_topic", default_value="/camera/depth/color/points"),
        DeclareLaunchArgument("rviz_config", default_value=default_config),
        Node(
            package="rviz2", executable="rviz2", name="terrain_rviz", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config"),
                       "-f", LaunchConfiguration("fixed_frame")],
            parameters=[{"use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool)}],
            remappings=[("/camera/depth/color/points", LaunchConfiguration("input_pointcloud_topic"))],
        ),
    ])
