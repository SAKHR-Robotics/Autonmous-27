import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    erc_path_planner_dir = get_package_share_directory('erc_path_planner')

    use_slam_str = LaunchConfiguration('use_slam').perform(context)
    use_slam = use_slam_str.lower() in ['true', '1']

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

    # When use_slam is True, map_server is disabled (RTAB-Map owns /map)
    # When use_slam is False, map_server runs dummy_map.yaml for standalone testing
    if use_slam:
        lifecycle_nodes = [
            'planner_server',
            'controller_server',
            'behavior_server',
            'bt_navigator',
            'velocity_smoother'
        ]
    else:
        lifecycle_nodes = [
            'map_server',
            'planner_server',
            'controller_server',
            'behavior_server',
            'bt_navigator',
            'velocity_smoother'
        ]

    nodes = []

    # 1. Map Server (only started if use_slam is False for standalone testing)
    if not use_slam:
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
        nodes.append(map_server_node)

    # 2. Planner Server (Smac Hybrid A*)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    nodes.append(planner_server_node)

    # 3. Controller Server (MPPI Controller)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    nodes.append(controller_server_node)

    # 4. Behavior Server (Recoveries: Spin, Backup, Wait)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    nodes.append(behavior_server_node)

    # 5. Behavior Tree Navigator
    bt_navigator_params = [
        LaunchConfiguration('params_file'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
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
    nodes.append(bt_navigator_node)

    # 6. Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
    )
    nodes.append(velocity_smoother_node)

    # 7. Unified Lifecycle Manager (boots core navigation nodes together)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'node_names': lifecycle_nodes,
                'bond_timeout': 0.0
            }
        ]
    )
    nodes.append(lifecycle_manager_node)

    # 8. Costmap Bridge Node (from Perception Obstacle Features)
    bridge_node = Node(
        package='erc_path_planner',
        executable='costmap_bridge_node',
        name='costmap_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    nodes.append(bridge_node)

    return nodes


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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='If true, map is provided by SLAM (RTAB-Map) and map_server is disabled. If false, map_server loads dummy_map.yaml for standalone testing.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to nav2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file to load (only used when use_slam:=false)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        OpaqueFunction(function=launch_setup)
    ])
