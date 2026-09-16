"""Launch the clean downstream marker interface only; Stage 4 tracking (and,
for base_link resolution, a TF tree with base_link <- camera_optical_frame)
must already be available."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("marker_detection"), "config", "marker_action_interface.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(package="marker_detection", executable="marker_action_interface_node",
             name="marker_action_interface", output="screen",
             parameters=[LaunchConfiguration("params_file"), {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
