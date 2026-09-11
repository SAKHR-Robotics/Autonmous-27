import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    erc_path_planner_dir = get_package_share_directory('erc_path_planner')

    default_params_file = os.path.join(
        erc_path_planner_dir,
        'config',
        'nav2_params.yaml'
    )

    default_map = os.path.join(
        erc_path_planner_dir,
        'config',
        'dummy_map.yaml'
    )

    # Locate Nav2 behavior trees from local package or installed ROS distro
    local_nav_to_pose = os.path.join(
        erc_path_planner_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    local_nav_through_poses = os.path.join(
        erc_path_planner_dir, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    if os.path.exists(local_nav_to_pose):
        default_nav_to_pose = local_nav_to_pose
        default_nav_through_poses = local_nav_through_poses if os.path.exists(local_nav_through_poses) else ''
    else:
        try:
            nav2_bt_dir = get_package_share_directory('nav2_bt_navigator')
            default_nav_to_pose = os.path.join(
                nav2_bt_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
            )
            default_nav_through_poses = os.path.join(
                nav2_bt_dir, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'
            )
        except Exception:
            default_nav_to_pose = ''
            default_nav_through_poses = ''

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to nav2 parameters file to use'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother'
    ]

    # 1. Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # 2. Planner Server (Smac Hybrid A*)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # 3. Controller Server (MPPI Controller)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # 4. Behavior Server (Recoveries: Spin, Backup, Wait)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # 5. Behavior Tree Navigator
    bt_navigator_params = [
        LaunchConfiguration('params_file')
    ]
    if default_nav_to_pose:
        bt_navigator_params.append({
            'default_nav_to_pose_bt_xml': default_nav_to_pose,
            'default_nav_through_poses_bt_xml': default_nav_through_poses
        })

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=bt_navigator_params
    )

    # 6. Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
    )

    # 7. Unified Lifecycle Manager (boots all 6 core navigation nodes together)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 0.0
            }
        ]
    )

    # 8. Costmap Bridge Node (from Tasbh-Tasks)
    bridge_node = Node(
        package='erc_path_planner',
        executable='costmap_bridge_node',
        name='costmap_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        params_file_arg,
        map_arg,
        use_sim_time_arg,
        autostart_arg,
        map_server_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        lifecycle_manager_node,
        bridge_node
    ])
