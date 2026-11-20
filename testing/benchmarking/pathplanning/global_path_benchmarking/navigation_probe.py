"""Request-correlated Nav2 probes. No path callback is treated as goal completion."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml

from global_path_benchmarking.report_generator import calculate_control_metrics
from global_path_benchmarking.schema import build_provenance, build_trial_record, gate


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _load_truth_occupancy(map_file, allow_unknown=False):
    spec = yaml.safe_load(Path(map_file).read_text())
    resolution = float(spec['resolution'])
    if resolution <= 0 or not math.isfinite(resolution):
        raise ValueError('Map resolution must be finite and positive')
    image_file = Path(spec['image'])
    if not image_file.is_absolute():
        image_file = Path(map_file).parent / image_file
    pixels = np.flipud(np.asarray(Image.open(image_file).convert('L')))
    occupancy = pixels / 255.0 if spec.get('negate', 0) else 1.0 - pixels / 255.0
    threshold = float(spec.get('occupied_thresh', 0.65) if allow_unknown else spec.get('free_thresh', 0.196))
    blocked = occupancy >= threshold
    origin = tuple(float(v) for v in spec['origin'])
    return blocked, occupancy, resolution, origin


def _to_se2(path):
    if not path:
        return []
    if len(path[0]) >= 3:
        return [(float(p[0]), float(p[1]), float(p[2])) for p in path]
    poses = []
    for idx, pt in enumerate(path):
        if idx < len(path) - 1:
            heading = math.atan2(path[idx + 1][1] - pt[1], path[idx + 1][0] - pt[0])
        elif poses:
            heading = poses[-1][2]
        else:
            heading = 0.0
        poses.append((float(pt[0]), float(pt[1]), heading))
    return poses


def _interpolate_se2_samples(poses, resolution, length, width):
    samples = []
    radius = math.hypot(length / 2, width / 2)
    for a, b in zip(poses, poses[1:]):
        da = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        travel = math.hypot(b[0] - a[0], b[1] - a[1]) + radius * abs(da)
        count = max(1, math.ceil(travel / (resolution / 4)))
        padding = travel / count / 2
        samples.extend(
            (
                a[0] + (b[0] - a[0]) * i / count,
                a[1] + (b[1] - a[1]) * i / count,
                a[2] + da * i / count,
                padding,
            )
            for i in range(count + 1)
        )
    if not samples:
        samples = [(*poses[0], 0.0)]
    return samples


def swept_collision(path, map_file, length=0.9, width=0.7, allow_unknown=False):
    """Conservative rectangle sweep against independent occupancy cells.

    Interpolate SE2 segments at <= quarter-cell motion, enlarge each rectangle
    by the maximum motion to its nearest sample, and test cell overlap by SAT.
    Unknown/outside cells are blocked unless allow_unknown=True. A returned False
    is relative to this supplied 2D map/footprint only; it is not a physical
    safety certification.
    """
    if not path or not all(math.isfinite(v) for p in path for v in p):
        return None
    blocked, _, resolution, (ox, oy, angle) = _load_truth_occupancy(map_file, allow_unknown=allow_unknown)
    height, cols = blocked.shape
    co, si = math.cos(angle), math.sin(angle)
    se2 = _to_se2(path)
    poses = [
        (co * (x - ox) + si * (y - oy), -si * (x - ox) + co * (y - oy), a - angle)
        for x, y, a in se2
    ]
    samples = _interpolate_se2_samples(poses, resolution, length, width)
    for x, y, a, pad in samples:
        c, s = math.cos(a), math.sin(a)
        hx, hy = length / 2 + pad, width / 2 + pad
        ex, ey = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        left, right = math.floor((x - ex) / resolution), math.floor((x + ex) / resolution)
        bottom, top = math.floor((y - ey) / resolution), math.floor((y + ey) / resolution)
        if left < 0 or bottom < 0 or right >= cols or top >= height:
            return True
        for row in range(bottom, top + 1):
            for col in range(left, right + 1):
                if not blocked[row, col]:
                    continue
                dx, dy = (col + .5) * resolution - x, (row + .5) * resolution - y
                # Separating axes: world XY and rectangle's local XY.
                cell_half = resolution / 2
                if (
                    abs(dx) <= ex + cell_half
                    and abs(dy) <= ey + cell_half
                    and abs(c * dx + s * dy) <= hx + cell_half * (abs(c) + abs(s))
                    and abs(-s * dx + c * dy) <= hy + cell_half * (abs(c) + abs(s))
                ):
                    return True
    return False


def swept_min_clearance(path, map_file, length=0.9, width=0.7, allow_unknown=False):
    """Minimum distance (meters) from swept footprint to occupied truth cells."""
    collided = swept_collision(path, map_file, length=length, width=width, allow_unknown=allow_unknown)
    if collided is None:
        return None
    if collided is True:
        return 0.0
    blocked, _, resolution, (ox, oy, angle) = _load_truth_occupancy(map_file, allow_unknown=allow_unknown)
    obs_rows, obs_cols = np.where(blocked)
    if len(obs_rows) == 0:
        return None
    obs_x = (obs_cols.astype(np.float64) + 0.5) * resolution
    obs_y = (obs_rows.astype(np.float64) + 0.5) * resolution

    co, si = math.cos(angle), math.sin(angle)
    se2 = _to_se2(path)
    poses = [
        (co * (x - ox) + si * (y - oy), -si * (x - ox) + co * (y - oy), a - angle)
        for x, y, a in se2
    ]
    samples = _interpolate_se2_samples(poses, resolution, length, width)
    hx, hy = length / 2.0, width / 2.0
    cell_half = resolution / 2.0
    min_clear = float('inf')

    for x, y, a, _ in samples:
        c, s = math.cos(a), math.sin(a)
        dx = obs_x - x
        dy = obs_y - y
        dx_loc = c * dx + s * dy
        dy_loc = -s * dx + c * dy
        qx = np.maximum(0.0, np.abs(dx_loc) - hx)
        qy = np.maximum(0.0, np.abs(dy_loc) - hy)
        dists = np.hypot(qx, qy) - cell_half
        local_min = float(np.min(dists))
        if local_min < min_clear:
            min_clear = local_min
    return max(0.0, min_clear) if math.isfinite(min_clear) else None


def sample_truth_map_cost(path, map_file):
    """Sample mean and integrated occupancy cost [0, 100] on independent truth map."""
    if not path or not all(math.isfinite(v) for p in path for v in p):
        return None, None
    _, occupancy, resolution, (ox, oy, angle) = _load_truth_occupancy(map_file, allow_unknown=False)
    height, cols = occupancy.shape
    co, si = math.cos(angle), math.sin(angle)
    total_cost = 0.0
    for pt in path:
        lx = co * (pt[0] - ox) + si * (pt[1] - oy)
        ly = -si * (pt[0] - ox) + co * (pt[1] - oy)
        col = math.floor(lx / resolution)
        row = math.floor(ly / resolution)
        if 0 <= row < height and 0 <= col < cols:
            total_cost += float(occupancy[row, col]) * 100.0
        else:
            total_cost += 100.0
    return total_cost / len(path), total_cost


def evaluate_plan_geometry(
    path_se2,
    *,
    start_xy=None,
    goal_xy=None,
    truth_map=None,
    reference_path=None,
    length=0.9,
    width=0.7,
    allow_unknown=False,
    start_tol=0.35,
    goal_tol=0.50,
):
    """Compute physical SI planner metrics and geometry validity without inventing defaults."""
    if not path_se2 or len(path_se2) < 2 or not all(math.isfinite(v) for p in path_se2 for v in p):
        return {
            'valid_geometry': False,
            'path_length_m': None,
            'euclidean_dist_m': None,
            'euclidean_length_ratio': None,
            'reference_length_m': None,
            'reference_length_ratio': None,
            'start_error_m': None,
            'goal_error_m': None,
            'endpoints_valid': False if path_se2 is not None else None,
            'cusp_count': None,
            'min_angle_deg': None,
            'max_angle_deg': None,
            'avg_angle_deg': None,
            'min_turn_radius_m': None,
            'avg_turn_radius_m': None,
            'swept_collision': None,
            'swept_min_clearance_m': None,
            'avg_cost': None,
            'total_cost': None,
        }

    se2 = _to_se2(path_se2)
    xy = [(p[0], p[1]) for p in se2]
    path_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:]))

    s_xy = tuple(start_xy[:2]) if start_xy is not None else xy[0]
    g_xy = tuple(goal_xy[:2]) if goal_xy is not None else xy[-1]
    euc_dist = math.hypot(g_xy[0] - s_xy[0], g_xy[1] - s_xy[1])
    euc_ratio = path_len / euc_dist if euc_dist > 1e-6 else None

    ref_len = None
    ref_ratio = None
    if reference_path and len(reference_path) >= 2 and all(
        math.isfinite(p[0]) and math.isfinite(p[1]) for p in reference_path
    ):
        ref_len = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(reference_path, reference_path[1:])
        )
        if ref_len > 1e-6:
            ref_ratio = path_len / ref_len

    start_err = math.hypot(xy[0][0] - start_xy[0], xy[0][1] - start_xy[1]) if start_xy is not None else None
    goal_err = math.hypot(xy[-1][0] - goal_xy[0], xy[-1][1] - goal_xy[1]) if goal_xy is not None else None
    if start_err is None and goal_err is None:
        endpoints_valid = None
    else:
        endpoints_valid = (
            (start_err is None or start_err <= start_tol)
            and (goal_err is None or goal_err <= goal_tol)
        )

    cusps = 0
    angles = []
    radii = []
    for a, b, c in zip(xy, xy[1:], xy[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        m1 = math.hypot(*v1)
        m2 = math.hypot(*v2)
        if m1 < 1e-6 or m2 < 1e-6:
            continue
        cos_theta = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)))
        theta_deg = math.degrees(math.acos(cos_theta))
        if theta_deg > 120.0:
            cusps += 1
            continue
        angles.append(theta_deg)
        area = 0.5 * abs(a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        if area > 1e-5 and theta_deg > 1.0:
            ac = math.hypot(c[0] - a[0], c[1] - a[1])
            rad = (m1 * m2 * ac) / (4.0 * area)
            if rad < 100.0:
                radii.append(rad)

    collision = None
    clearance = None
    avg_cost = None
    total_cost = None
    if truth_map:
        collision = swept_collision(se2, truth_map, length=length, width=width, allow_unknown=allow_unknown)
        clearance = swept_min_clearance(se2, truth_map, length=length, width=width, allow_unknown=allow_unknown)
        avg_cost, total_cost = sample_truth_map_cost(se2, truth_map)

    return {
        'valid_geometry': True,
        'path_length_m': path_len,
        'euclidean_dist_m': euc_dist,
        'euclidean_length_ratio': euc_ratio,
        'reference_length_m': ref_len,
        'reference_length_ratio': ref_ratio,
        'start_error_m': start_err,
        'goal_error_m': goal_err,
        'endpoints_valid': endpoints_valid,
        'cusp_count': cusps,
        'min_angle_deg': min(angles) if angles else 0.0,
        'max_angle_deg': max(angles) if angles else 0.0,
        'avg_angle_deg': float(np.mean(angles)) if angles else 0.0,
        'min_turn_radius_m': min(radii) if radii else None,
        'avg_turn_radius_m': float(np.mean(radii)) if radii else None,
        'swept_collision': collision,
        'swept_min_clearance_m': clearance,
        'avg_cost': avg_cost,
        'total_cost': total_cost,
    }


def _parse_waypoints(raw_waypoints):
    parsed = []
    for item in raw_waypoints:
        parts = [float(p.strip()) for p in item.split(',')]
        if len(parts) not in (2, 3) or not all(math.isfinite(v) for v in parts):
            raise ValueError(f'Invalid waypoint {item!r}; expected x,y[,yaw]')
        parsed.append((parts[0], parts[1], parts[2] if len(parts) == 3 else 0.0))
    return parsed


def _make_pose(node, x, y, heading):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(heading / 2.0)
    pose.pose.orientation.w = math.cos(heading / 2.0)
    return pose


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path as NavPath
    from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
    from tf2_ros import Buffer, TransformListener

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['plan', 'navigate', 'navigate_through_poses'], default='plan')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--waypoints', nargs='*', default=[], help='x,y,yaw waypoints for navigate_through_poses')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--cancel-after', type=float)
    parser.add_argument('--truth-map', help='Independent scenario map YAML, not a Nav2 output')
    parser.add_argument('--allow-unknown', action='store_true')
    parser.add_argument('--expected-infeasible', action='store_true')
    parser.add_argument('--odom-topic', default='/odometry/filtered')
    parser.add_argument('--truth-odom-topic', default='')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--footprint-length', type=float, default=0.9)
    parser.add_argument('--footprint-width', type=float, default=0.7)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--output', required=True)
    options = parser.parse_args(args)

    waypoints = _parse_waypoints(options.waypoints) if options.waypoints else []
    if options.mode == 'navigate_through_poses':
        if not waypoints:
            parser.error('--waypoints is required when --mode navigate_through_poses')
        target_goal = list(waypoints[-1])
    else:
        if options.goal is None:
            parser.error('--goal is required for --mode plan/navigate')
        target_goal = list(options.goal)

    if (
        not math.isfinite(options.timeout)
        or options.timeout <= 0
        or (options.cancel_after is not None and (not math.isfinite(options.cancel_after) or options.cancel_after <= 0))
        or not all(math.isfinite(v) for v in target_goal)
        or (options.start is not None and not all(math.isfinite(v) for v in options.start))
        or not math.isfinite(options.footprint_length) or options.footprint_length <= 0
        or not math.isfinite(options.footprint_width) or options.footprint_width <= 0
    ):
        parser.error('Finite positive limits, footprint dimensions, and finite goal/start are required')

    rclpy.init()
    node = Node('navigation_probe', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=options.use_sim_time)])
    transforms = Buffer()
    TransformListener(transforms, node)
    odometry, truth_odometry, commands, plans = [], [], [], []
    node.create_subscription(Odometry, options.odom_topic, lambda m: odometry.append((time.monotonic(), m)), 100)
    if options.truth_odom_topic:
        node.create_subscription(Odometry, options.truth_odom_topic, lambda m: truth_odometry.append((time.monotonic(), m)), 100)
    node.create_subscription(Twist, options.cmd_vel_topic, lambda m: commands.append((time.monotonic(), m)), 100)
    node.create_subscription(NavPath, '/plan', lambda m: plans.append((time.monotonic(), m)), 20)

    action_map = {
        'plan': (ComputePathToPose, 'compute_path_to_pose'),
        'navigate': (NavigateToPose, 'navigate_to_pose'),
        'navigate_through_poses': (NavigateThroughPoses, 'navigate_through_poses'),
    }
    action, action_name = action_map[options.mode]
    client = ActionClient(node, action, action_name)

    record = {
        'mode': options.mode,
        'goal': target_goal,
        'terminal': 'UNAVAILABLE',
        'action_status': None,
        'error_code': None,
        'swept_collision': None,
        'swept_min_clearance_m': None,
        'executed_swept_collision': None,
        'executed_min_clearance_m': None,
        'position_error_m': None,
        'motion_source': options.odom_topic,
        'truth_map': options.truth_map,
        'timing_basis': 'monotonic end-to-end action latency',
    }
    if waypoints:
        record['waypoints'] = [list(w) for w in waypoints]

    def wait(future, seconds):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        return future.result() if future.done() else None

    started = time.monotonic()
    try:
        if not client.wait_for_server(timeout_sec=15):
            record['terminal'] = 'SERVER_UNAVAILABLE'
        else:
            goal = action.Goal()
            if options.mode == 'plan':
                goal.goal = _make_pose(node, *target_goal)
                goal.planner_id = 'GridBased'
                if options.start is not None:
                    goal.start = _make_pose(node, *options.start)
                    goal.use_start = True
                else:
                    goal.use_start = False
            elif options.mode == 'navigate':
                goal.pose = _make_pose(node, *target_goal)
            else:
                goal.poses = [_make_pose(node, *wp) for wp in waypoints]

            started = time.monotonic()
            handle = wait(client.send_goal_async(goal), 5)
            if handle is None or not handle.accepted:
                record['terminal'] = 'NO_ACCEPTANCE' if handle is None else 'REJECTED'
            else:
                record['goal_id'] = bytes(handle.goal_id.uuid).hex()
                future = handle.get_result_async()
                limit = options.cancel_after if options.cancel_after is not None else options.timeout
                result = wait(future, min(limit, options.timeout))
                if result is None:
                    record['termination_trigger'] = 'USER_CANCEL' if options.cancel_after is not None else 'DEADLINE'
                    record['terminal'] = 'CANCEL_REQUESTED' if options.cancel_after is not None else 'TIMEOUT'
                    wait(handle.cancel_goal_async(), 5)
                    result = wait(future, 5)
                action_elapsed = time.monotonic() - started
                if result is not None:
                    record['action_status'] = result.status
                    if record['terminal'] in ('UNAVAILABLE', 'CANCEL_REQUESTED'):
                        record['terminal'] = {
                            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                            GoalStatus.STATUS_ABORTED: 'ABORTED',
                            GoalStatus.STATUS_CANCELED: 'CANCELED',
                        }.get(result.status, 'OTHER')
                    record['error_code'] = getattr(result.result, 'error_code', None)
                    if options.mode == 'plan':
                        record['path'] = [
                            [p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                            for p in result.result.path.poses
                        ]
                        if record['path']:
                            record['endpoint_error_m'] = math.hypot(
                                record['path'][-1][0] - target_goal[0],
                                record['path'][-1][1] - target_goal[1],
                            )
                record['elapsed_sec'] = action_elapsed

                # Extract plan geometry (either direct ComputePathToPose output or latest /plan topic)
                active_path = record.get('path')
                if active_path is None and plans:
                    active_path = [
                        [p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                        for p in plans[-1][1].poses
                    ]
                if active_path:
                    plan_geom = evaluate_plan_geometry(
                        active_path,
                        start_xy=options.start[:2] if options.start else None,
                        goal_xy=target_goal[:2],
                        truth_map=options.truth_map,
                        length=options.footprint_length,
                        width=options.footprint_width,
                        allow_unknown=options.allow_unknown,
                    )
                    record['swept_collision'] = plan_geom['swept_collision']
                    record['swept_min_clearance_m'] = plan_geom['swept_min_clearance_m']
                    record['plan_metrics'] = plan_geom

                trace_source = truth_odometry if truth_odometry else odometry
                if odometry:
                    record['odometry_frame'] = odometry[-1][1].header.frame_id
                    try:
                        location = transforms.lookup_transform('map', 'base_link', Time()).transform.translation
                        record['position_error_m'] = math.hypot(location.x - target_goal[0], location.y - target_goal[1])
                        record['position_error_basis'] = 'estimated map TF, not independent localization truth'
                    except Exception as error:
                        record['position_error_unavailable'] = str(error)
                    record['odometry_trace'] = [
                        [t - started, m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)]
                        for t, m in odometry
                    ]
                if trace_source and options.truth_map:
                    exec_se2 = [
                        (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
                        for _, m in trace_source
                    ]
                    record['executed_swept_collision'] = swept_collision(
                        exec_se2,
                        options.truth_map,
                        length=options.footprint_length,
                        width=options.footprint_width,
                        allow_unknown=options.allow_unknown,
                    )
                    record['executed_min_clearance_m'] = swept_min_clearance(
                        exec_se2,
                        options.truth_map,
                        length=options.footprint_length,
                        width=options.footprint_width,
                        allow_unknown=options.allow_unknown,
                    )

                record['cmd_vel_publishers'] = node.count_publishers(options.cmd_vel_topic)
                record['command_count'] = len(commands)
                record['reverse_command_count'] = sum(m.linear.x < -0.01 for _, m in commands)
                record['min_linear_mps'] = min((m.linear.x for _, m in commands), default=None)
                record['peak_linear_mps'] = max((abs(m.linear.x) for _, m in commands), default=None)
                record['peak_angular_rps'] = max((abs(m.angular.z) for _, m in commands), default=None)
                record['cmd_bounds_ok'] = (
                    all(
                        -0.50 - 1e-4 <= m.linear.x <= 1.00 + 1e-4
                        and abs(m.linear.y) <= 0.01 + 1e-4
                        and abs(m.angular.z) <= 1.00 + 1e-4
                        for _, m in commands
                    )
                    if commands
                    else None
                )

                # Controller metrics on full duration when navigating
                if options.mode in ('navigate', 'navigate_through_poses'):
                    cmd_rows = [(t, m.linear.x, m.linear.y, m.angular.z) for t, m in commands]
                    odom_rows = [
                        (
                            t,
                            m.pose.pose.position.x,
                            m.pose.pose.position.y,
                            yaw(m.pose.pose.orientation),
                            m.twist.twist.linear.x,
                            m.twist.twist.angular.z,
                        )
                        for t, m in trace_source
                    ]
                    plan_xy = [(p[0], p[1]) for p in (active_path or [])]
                    goal_msg = _make_pose(node, *target_goal)
                    record['control_metrics'] = calculate_control_metrics(
                        cmd_rows,
                        odom_rows,
                        plan_xy,
                        goal_msg,
                        execution_complete=(record['terminal'] == 'SUCCEEDED'),
                    )

        # Build embedded v2.0.0 schema record
        dep_manifest = {
            'map': 'MOCK' if options.truth_map else 'REAL',
            'localization': 'MOCK' if 'mock' in options.odom_topic else 'REAL',
            'perception': 'MOCK',
            'rover_plant': 'MOCK' if not options.truth_odom_topic else 'GAZEBO',
            'clock': 'SIM' if options.use_sim_time else 'WALL',
        }
        eval_mode = 'planner_only' if options.mode == 'plan' else 'nav_closed_loop'
        prov = build_provenance(
            evaluation_mode=eval_mode,
            dependency_manifest=dep_manifest,
            truth_source='independent_scenario_yaml' if options.truth_map else 'estimated_tf_only',
            claim_scope='Single-shot probe execution; see dependency_manifest for real vs mocked components.',
            seed=options.seed,
            use_sim_time=options.use_sim_time,
        )
        gates = {}
        if options.expected_infeasible:
            rejected = record['terminal'] == 'ABORTED' and not record.get('path')
            gates['expected_infeasible_rejected'] = gate(
                rejected if record['terminal'] not in ('UNAVAILABLE', 'SERVER_UNAVAILABLE') else None,
                f"terminal={record['terminal']}, path_len={len(record.get('path') or [])}",
            )
            cat = 'negative_infeasible'
        elif options.cancel_after is not None:
            gates['action_canceled'] = gate(
                record['terminal'] == 'CANCELED' if record['terminal'] != 'SERVER_UNAVAILABLE' else None,
                f"terminal={record['terminal']}",
            )
            cat = 'safety_cancel'
        else:
            gates['action_succeeded'] = gate(
                True if record['terminal'] == 'SUCCEEDED' else (None if record['terminal'] in ('SERVER_UNAVAILABLE', 'UNAVAILABLE') else False),
                f"terminal={record['terminal']}, error_code={record.get('error_code')}",
            )
            if options.truth_map and record.get('swept_collision') is not None:
                gates['swept_collision_free'] = gate(
                    record['swept_collision'] is False,
                    f"swept_min_clearance_m={record.get('swept_min_clearance_m')}",
                )
            if options.mode != 'plan' and options.truth_map and record.get('executed_swept_collision') is not None:
                gates['executed_swept_collision_free'] = gate(
                    record['executed_swept_collision'] is False,
                    f"executed_min_clearance_m={record.get('executed_min_clearance_m')}",
                )
            if options.mode != 'plan' and record.get('cmd_bounds_ok') is not None:
                gates['cmd_vel_within_limits'] = gate(record['cmd_bounds_ok'], 'nav2_params limits')
            cat = 'positive_static'

        v2_metrics = {
            'elapsed_sec': record.get('elapsed_sec'),
            'swept_min_clearance_m': record.get('swept_min_clearance_m'),
            'executed_min_clearance_m': record.get('executed_min_clearance_m'),
            'position_error_m': record.get('position_error_m'),
            'endpoint_error_m': record.get('endpoint_error_m'),
            'command_count': record.get('command_count'),
            'reverse_command_count': record.get('reverse_command_count'),
            'peak_linear_mps': record.get('peak_linear_mps'),
            'peak_angular_rps': record.get('peak_angular_rps'),
        }
        if 'plan_metrics' in record:
            v2_metrics.update({
                k: v for k, v in record['plan_metrics'].items()
                if k not in ('valid_geometry', 'endpoints_valid', 'swept_collision')
            })
        if 'control_metrics' in record:
            v2_metrics.update({
                k: v for k, v in record['control_metrics'].items()
                if k != 'controller_score'
            })

        record['schema_v2'] = build_trial_record(
            provenance=prov,
            scenario_id=Path(options.truth_map).stem if options.truth_map else f'probe_{options.mode}',
            category=cat,
            expected_outcome='PASS',
            gates=gates,
            metrics=v2_metrics,
            terminal_state='TIMEOUT' if record['terminal'] == 'TIMEOUT' else None,
            blocked_reason='Action server unavailable' if record['terminal'] == 'SERVER_UNAVAILABLE' else None,
        )
    finally:
        Path(options.output).parent.mkdir(parents=True, exist_ok=True)
        Path(options.output).write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
        summary = {k: v for k, v in record.items() if k not in ('path', 'odometry_trace', 'schema_v2')}
        print(json.dumps(summary))
        node.destroy_node()
        rclpy.try_shutdown()

    if options.expected_infeasible:
        return 0 if record['terminal'] == 'ABORTED' and not record.get('path') else 1
    return 0 if record['terminal'] == 'SUCCEEDED' and record['swept_collision'] is not True and record['executed_swept_collision'] is not True else 1


if __name__ == '__main__':
    raise SystemExit(main())
