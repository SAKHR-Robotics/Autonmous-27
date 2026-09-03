"""Launch the complete six-stage Marker Detection Branch as one system.

Composes the three existing per-stage launch files rather than duplicating
their node definitions:

    marker_detection.launch.py  -> Steps 1-4 (detection, 3D position, pose, tracking)
    marker_tf.launch.py         -> Step 5 (camera -> marker TF)
    marker_mapping.launch.py    -> Step 6 (global marker map)
    marker_action_interface.launch.py -> clean downstream manipulation/planning interface

The RealSense driver is intentionally NOT started here; the project assumes
the camera is already running (see README.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("marker_detection")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    rgb_topic_arg = DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw")
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/depth/image_raw")
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/camera_info")

    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "marker_detection.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        }.items())

    marker_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "marker_tf.launch.py")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items())

    marker_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "marker_mapping.launch.py")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items())

    marker_action_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "marker_action_interface.launch.py")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items())

    return LaunchDescription([
        use_sim_time_arg, rgb_topic_arg, depth_topic_arg, camera_info_topic_arg,
        detection, marker_tf, marker_mapping, marker_action_interface,
    ])
