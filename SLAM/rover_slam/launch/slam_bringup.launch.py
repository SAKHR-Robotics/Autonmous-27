import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    launch_camera = LaunchConfiguration('launch_camera')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup nav2 lifecycle nodes'
    )
    declare_launch_camera = DeclareLaunchArgument(
        'launch_camera',
        default_value='false',
        description='Whether to launch the physical RealSense camera driver (set true for physical robot, false for simulation/rosbag)'
    )
    declare_rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/image_raw',
        description='RGB Image topic name'
    )
    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Depth Image topic name'
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Camera Info topic name'
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'static_transforms.launch.py'))
    )
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'vision_helper.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_camera)
    )
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic
        }.items()
    )
    costmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'costmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'standalone': 'false'
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_launch_camera,
        declare_rgb_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        static_tf_launch,
        ekf_launch,
        vision_launch,
        rtabmap_launch,
        costmap_launch
    ])
