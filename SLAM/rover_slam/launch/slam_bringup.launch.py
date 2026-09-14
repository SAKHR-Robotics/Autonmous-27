import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_static_tf = LaunchConfiguration('launch_static_tf')
    standalone_tf = LaunchConfiguration('standalone_tf')
    launch_costmap_stub = LaunchConfiguration('launch_costmap_stub')
    launch_aruco_stub = LaunchConfiguration('launch_aruco_stub')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')

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
    declare_launch_static_tf = DeclareLaunchArgument(
        'launch_static_tf',
        default_value='true',
        description='Whether to include static_transforms.launch.py'
    )
    declare_standalone_tf = DeclareLaunchArgument(
        'standalone_tf',
        default_value='false',
        description='If true, publish base_link sensor TFs in static_transforms (set true only if robot_state_publisher is NOT active)'
    )
    declare_launch_costmap_stub = DeclareLaunchArgument(
        'launch_costmap_stub',
        default_value='false',
        description='Whether to run costmap_test_stub node to simulate obstacle points for costmap verification'
    )
    declare_launch_aruco_stub = DeclareLaunchArgument(
        'launch_aruco_stub',
        default_value='true',
        description='Whether to run mock_aruco_publisher to simulate ArUco landmark detection'
    )
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Whether to launch RViz2 visualization dashboard'
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
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Raw odometry topic (e.g. /odom for Gazebo, /wheel/odom_raw for physical robot)'
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'static_transforms.launch.py')),
        launch_arguments={
            'standalone': standalone_tf
        }.items(),
        condition=IfCondition(launch_static_tf)
    )
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'ekf.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic
        }.items()
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
    costmap_stub_node = Node(
        package='rover_slam',
        executable='costmap_test_stub',
        name='costmap_test_stub',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_costmap_stub)
    )
    aruco_stub_node = Node(
        package='rover_slam',
        executable='mock_aruco_publisher',
        name='mock_aruco_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_aruco_stub)
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rviz.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_rviz)
    )

    delayed_costmap_actions = TimerAction(
        period=3.0,
        actions=[costmap_launch, costmap_stub_node]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_launch_camera,
        declare_launch_static_tf,
        declare_standalone_tf,
        declare_launch_costmap_stub,
        declare_launch_aruco_stub,
        declare_launch_rviz,
        declare_rgb_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        declare_odom_topic,
        static_tf_launch,
        ekf_launch,
        vision_launch,
        rtabmap_launch,
        delayed_costmap_actions,
        aruco_stub_node,
        rviz_launch
    ])

