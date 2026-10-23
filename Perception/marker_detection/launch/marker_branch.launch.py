"""Launch the complete real marker branch; camera and localization are external.

This existing entry point retains global mapping by default. Use
perception_bringup for selectable nodes and mapping off by default.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("marker_detection")
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
    ]
    stages = [
        ("detection", "marker_detection"),
        ("tf", "marker_tf"),
        ("mapping", "marker_mapping"),
        ("action", "marker_action_interface"),
    ]
    groups = []
    for stage, stem in stages:
        argument = f"{stage}_params_file"
        arguments.append(DeclareLaunchArgument(
            argument, default_value=os.path.join(share, "config", f"{stem}.yaml")))
        forwarded = {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration(argument),
        }
        if stage == "detection":
            forwarded.update({name: LaunchConfiguration(name) for name in (
                "rgb_topic", "depth_topic", "camera_info_topic")})
        # Every child declares params_file. Scope prevents sibling leakage.
        groups.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", f"{stem}.launch.py")),
            launch_arguments=forwarded.items(),
        )]))
    return LaunchDescription(arguments + groups)
