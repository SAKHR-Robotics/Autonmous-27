"""Request-correlated Nav2 probes. No path callback is treated as goal completion."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def swept_collision(path, map_file, length=0.9, width=0.7):
    """Conservative rectangle sweep against independent occupancy cells.

    Interpolate SE2 segments at <= quarter-cell motion, enlarge each rectangle
    by the maximum motion to its nearest sample, and test cell overlap by SAT.
    Unknown/outside cells are blocked. A returned False is relative to this
    supplied 2D map/footprint only; it is not a physical safety certification.
    """
    if not path or not all(math.isfinite(v) for p in path for v in p):
        return None
    spec = yaml.safe_load(Path(map_file).read_text())
    resolution = float(spec['resolution'])
    if resolution <= 0 or not math.isfinite(resolution):
        raise ValueError('Map resolution must be finite and positive')
    image_file = Path(spec['image'])
    if not image_file.is_absolute():
        image_file = Path(map_file).parent / image_file
    pixels = np.flipud(np.asarray(Image.open(image_file).convert('L')))
    occupancy = pixels / 255.0 if spec.get('negate', 0) else 1.0 - pixels / 255.0
    blocked = occupancy >= float(spec.get('free_thresh', 0.196))
    height, cols = blocked.shape
    ox, oy, angle = spec['origin']
    co, si = math.cos(angle), math.sin(angle)
    poses = [(co*(x-ox)+si*(y-oy), -si*(x-ox)+co*(y-oy), a-angle)
             for x, y, a in path]
    samples = []
    radius = math.hypot(length/2, width/2)
    for a, b in zip(poses, poses[1:]):
        da = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        travel = math.hypot(b[0]-a[0], b[1]-a[1]) + radius*abs(da)
        count = max(1, math.ceil(travel/(resolution/4)))
        padding = travel/count/2
        samples.extend((a[0]+(b[0]-a[0])*i/count,
                        a[1]+(b[1]-a[1])*i/count, a[2]+da*i/count, padding)
                       for i in range(count+1))
    if not samples:
        samples = [(*poses[0], 0.0)]
    for x, y, a, pad in samples:
        c, s = math.cos(a), math.sin(a)
        hx, hy = length/2+pad, width/2+pad
        ex, ey = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
        left, right = math.floor((x-ex)/resolution), math.floor((x+ex)/resolution)
        bottom, top = math.floor((y-ey)/resolution), math.floor((y+ey)/resolution)
        if left < 0 or bottom < 0 or right >= cols or top >= height:
            return True
        for row in range(bottom, top+1):
            for col in range(left, right+1):
                if not blocked[row, col]:
                    continue
                dx, dy = (col+.5)*resolution-x, (row+.5)*resolution-y
                # Separating axes: world XY and rectangle's local XY.
                cell_half = resolution/2
                if (abs(dx) <= ex+cell_half and abs(dy) <= ey+cell_half
                        and abs(c*dx+s*dy) <= hx+cell_half*(abs(c)+abs(s))
                        and abs(-s*dx+c*dy) <= hy+cell_half*(abs(c)+abs(s))):
                    return True
    return False


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['plan', 'navigate'], default='plan')
    parser.add_argument('--goal', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--cancel-after', type=float)
    parser.add_argument('--truth-map', help='Independent scenario map YAML, not a Nav2 output')
    parser.add_argument('--odom-topic', default='/odometry/filtered')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--output', required=True)
    options = parser.parse_args(args)
    if (not math.isfinite(options.timeout) or options.timeout <= 0
            or options.cancel_after is not None and (not math.isfinite(options.cancel_after) or options.cancel_after <= 0)
            or not all(math.isfinite(v) for v in options.goal)):
        parser.error('Finite positive time limits and a finite goal are required')
    rclpy.init()
    node = Node('navigation_probe', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=options.use_sim_time)])
    transforms = Buffer()
    listener = TransformListener(transforms, node)
    odometry, commands = [], []
    node.create_subscription(Odometry, options.odom_topic,
        lambda m: odometry.append((time.monotonic(), m)), 100)
    node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append((time.monotonic(), m)), 100)
    action = ComputePathToPose if options.mode == 'plan' else NavigateToPose
    client = ActionClient(node, action, 'compute_path_to_pose' if options.mode == 'plan' else 'navigate_to_pose')
    record = {'mode': options.mode, 'goal': options.goal, 'terminal': 'UNAVAILABLE',
              'action_status': None, 'error_code': None, 'swept_collision': None,
              'position_error_m': None, 'motion_source': options.odom_topic,
              'truth_map': options.truth_map, 'timing_basis': 'monotonic end-to-end action latency'}

    def wait(future, seconds):
        end = time.monotonic()+seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        return future.result() if future.done() else None

    try:
        if not client.wait_for_server(timeout_sec=15):
            record['terminal'] = 'SERVER_UNAVAILABLE'
        else:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = options.goal[:2]
            pose.pose.orientation.z = math.sin(options.goal[2]/2)
            pose.pose.orientation.w = math.cos(options.goal[2]/2)
            goal = action.Goal()
            if options.mode == 'plan':
                goal.goal, goal.planner_id, goal.use_start = pose, 'GridBased', False
            else:
                goal.pose = pose
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
                action_elapsed = time.monotonic()-started
                if result is not None:
                    record['action_status'] = result.status
                    if record['terminal'] in ('UNAVAILABLE', 'CANCEL_REQUESTED'):
                        record['terminal'] = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                            GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(result.status, 'OTHER')
                    record['error_code'] = getattr(result.result, 'error_code', None)
                    if options.mode == 'plan':
                        record['path'] = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                                          for p in result.result.path.poses]
                        if record['path']:
                            record['endpoint_error_m'] = math.hypot(record['path'][-1][0]-options.goal[0], record['path'][-1][1]-options.goal[1])
                        if record['path'] and options.truth_map:
                            record['swept_collision'] = swept_collision(record['path'], options.truth_map)
                record['elapsed_sec'] = action_elapsed
                if odometry:
                    record['odometry_frame'] = odometry[-1][1].header.frame_id
                    try:
                        location = transforms.lookup_transform('map', 'base_link', Time()).transform.translation
                        record['position_error_m'] = math.hypot(location.x-options.goal[0], location.y-options.goal[1])
                        record['position_error_basis'] = 'estimated map TF, not independent localization truth'
                    except Exception as error:
                        record['position_error_unavailable'] = str(error)
                    record['odometry_trace'] = [[t-started, m.pose.pose.position.x,
                        m.pose.pose.position.y, yaw(m.pose.pose.orientation)] for t,m in odometry]
                record['command_count'] = len(commands)
                record['reverse_command_count'] = sum(m.linear.x < -0.01 for _,m in commands)
                record['min_linear_mps'] = min((m.linear.x for _,m in commands), default=None)
                record['peak_linear_mps'] = max((abs(m.linear.x) for _,m in commands), default=None)
                record['peak_angular_rps'] = max((abs(m.angular.z) for _,m in commands), default=None)
    finally:
        Path(options.output).parent.mkdir(parents=True, exist_ok=True)
        Path(options.output).write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in record.items() if k not in ('path','odometry_trace')}))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if record['terminal'] == 'SUCCEEDED' and record['swept_collision'] is not True else 1


if __name__ == '__main__':
    raise SystemExit(main())
