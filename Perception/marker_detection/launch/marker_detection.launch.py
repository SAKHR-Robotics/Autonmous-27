"""Launch the detector only; the RealSense driver is intentionally external."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(get_package_share_directory("marker_detection"), "config", "marker_detection.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        Node(package="marker_detection", executable="marker_detection_node", name="marker_detection",
             output="screen", parameters=[LaunchConfiguration("params_file"), {
                 "use_sim_time": LaunchConfiguration("use_sim_time"),
                 "rgb_topic": LaunchConfiguration("rgb_topic"),
                 "depth_topic": LaunchConfiguration("depth_topic"),
                 "camera_info_topic": LaunchConfiguration("camera_info_topic"),
             }]),
    ])
