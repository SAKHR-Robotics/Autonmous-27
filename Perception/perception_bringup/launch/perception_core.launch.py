"""Compose real perception nodes without camera drivers or test publishers."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    marker_share = get_package_share_directory("marker_detection")
    terrain_share = get_package_share_directory("terrain_geometry")
    bringup_share = get_package_share_directory("perception_bringup")
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("input_pointcloud_topic", default_value="/camera/depth/color/points"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz_fixed_frame", default_value=LaunchConfiguration("target_frame")),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            bringup_share, "rviz", "perception_view.rviz")),
    ]
    stages = [
        ("terrain", terrain_share, "terrain", "true"),
        ("marker_detection", marker_share, "marker_detection", "true"),
        ("marker_tf", marker_share, "marker_tf", "true"),
        ("marker_mapping", marker_share, "marker_mapping", "false"),
        ("marker_action", marker_share, "marker_action_interface", "true"),
    ]
    groups = []
    for stage, share, stem, enabled in stages:
        switch = f"enable_{stage}"
        arguments.append(DeclareLaunchArgument(switch, default_value=enabled, choices=["true", "false"]))
        forwarded = {"use_sim_time": LaunchConfiguration("use_sim_time")}
        if stage == "terrain":
            forwarded.update({name: LaunchConfiguration(name) for name in (
                "input_pointcloud_topic", "target_frame")})
        else:
            params_argument = f"{stage}_params_file"
            arguments.append(DeclareLaunchArgument(
                params_argument, default_value=os.path.join(share, "config", f"{stem}.yaml")))
            forwarded["params_file"] = LaunchConfiguration(params_argument)
            if stage == "marker_detection":
                forwarded.update({name: LaunchConfiguration(name) for name in (
                    "rgb_topic", "depth_topic", "camera_info_topic")})
        groups.append(GroupAction(
            scoped=True, condition=IfCondition(LaunchConfiguration(switch)),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", f"{stem}.launch.py")),
                launch_arguments=forwarded.items(),
            )],
        ))
    groups.append(GroupAction(
        scoped=True, condition=IfCondition(LaunchConfiguration("use_rviz")),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "rviz.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "fixed_frame": LaunchConfiguration("rviz_fixed_frame"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "input_pointcloud_topic": LaunchConfiguration("input_pointcloud_topic"),
            }.items(),
        )],
    ))
    return LaunchDescription(arguments + groups)
