import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    rtabmap_config_path = os.path.join(pkg_share, 'config', 'rtabmap.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
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
    declare_delete_db_on_start = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='true',
        description='Delete previous database on startup if true'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_delete_db_on_start,
        declare_rgb_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'],
            parameters=[
                rtabmap_config_path,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odom', '/odometry/filtered'),
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('landmark', '/perception/aruco_pose')
            ]
        )
    ])
