"""Launch only global marker mapping; localization and Step 5 TF must already run."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(get_package_share_directory("marker_detection"), "config", "marker_mapping.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(package="marker_detection", executable="marker_map_node", name="marker_map", output="screen",
             parameters=[LaunchConfiguration("params_file"), {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
