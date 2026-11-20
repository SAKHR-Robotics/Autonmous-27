"""Bounded nominal synthetic consistency/loss check; no physical accuracy verdict."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='Absolute JSON path outside source packages')
    options = parser.parse_args(args)
    output = Path(options.output)
    if not output.is_absolute():
        parser.error('--output must be absolute')
    rclpy.init(args=[])
    node = Node('local_estimator_probe')
    publisher = node.create_publisher(Twist, '/testing/slam/cmd_vel', 10)
    client = AsyncParameterClient(node, '/slam_sensor_adapter')
    samples = {'truth': [], 'estimate': [], 'raw': [], 'filtered': [], 'health': []}
    phase = 'initialization'
    elapsed = 0.

    def odometry(kind, message):
        q = message.pose.pose.orientation
        samples[kind].append(dict(phase=phase, elapsed=elapsed,
            stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            x=message.pose.pose.position.x, y=message.pose.pose.position.y,
            yaw=math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
            variance=float(message.twist.covariance[0])))

    subscriptions = []
    for kind, topic in [('truth', '/testing/slam/truth'), ('estimate', '/odometry/filtered'),
                        ('raw', '/wheel/odom_raw'), ('filtered', '/wheel/odom_filtered')]:
        subscriptions.append(node.create_subscription(Odometry, topic,
            lambda msg, key=kind: odometry(key, msg), 100))

    def health(message):
        for status in message.status:
            samples['health'].append(dict(phase=phase, elapsed=elapsed, message=status.message))
    subscriptions.append(node.create_subscription(DiagnosticArray, '/slam/slip_diagnostics', health, 100))
    scenarios = [('stationary', 2., 0., 0., False, False),
                 ('straight', 4., .2, 0., False, False),
                 ('turn', 4., .15, .3, False, False),
                 ('imu_loss', 2., .2, 0., True, False),
                 ('imu_recovery', 2., .2, 0., False, False),
                 ('wheel_loss', 2., .2, 0., False, True),
                 ('wheel_recovery', 2., .2, 0., False, False)]
    try:
        if not client.wait_for_services(timeout_sec=5.):
            raise RuntimeError('Start local_mocked.launch.py before this observer')
        for phase, duration, velocity, yaw_rate, drop_imu, drop_wheels in scenarios:
            elapsed = 0.
            future = client.set_parameters([Parameter('drop_imu', value=drop_imu),
                                            Parameter('drop_wheels', value=drop_wheels)])
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            if not future.done() or not all(result.successful for result in future.result().results):
                raise RuntimeError('Could not select sensor loss scenario')
            start = time.monotonic()
            while time.monotonic() - start < duration:
                elapsed = time.monotonic() - start
                command = Twist(); command.linear.x = velocity; command.angular.z = yaw_rate
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=.02)
        def settled(kind, selected):
            return [sample for sample in samples[kind] if sample['phase'] == selected and sample['elapsed'] > 1.]
        loss = settled('filtered', 'imu_loss')
        def healthy(selected):
            observations = settled('health', selected)
            return bool(observations) and all(s['message'] == 'healthy' for s in observations)
        contracts = {
            'mock_truth_observed': bool(samples['truth']),
            'all_real_odometry_stages_observed': all(samples[key] for key in ('raw', 'filtered', 'estimate')),
            'imu_loss_inflates_covariance': bool(loss) and min(s['variance'] for s in loss) >= 2.,
            'imu_recovery_is_healthy': healthy('imu_recovery'),
            'missing_wheels_stop_raw_output': not settled('raw', 'wheel_loss'),
            'missing_wheels_stop_filtered_wheel_output': not settled('filtered', 'wheel_loss'),
            'wheel_recovery_resumes_raw_output': bool(settled('raw', 'wheel_recovery')),
            'wheel_recovery_is_healthy': healthy('wheel_recovery'),
        }
        matched = []
        for estimate in samples['estimate']:
            if estimate['phase'] not in ('stationary', 'straight', 'turn') or not samples['truth']:
                continue
            truth = min(samples['truth'], key=lambda value: abs(value['stamp']-estimate['stamp']))
            if abs(truth['stamp'] - estimate['stamp']) <= .05:
                matched.append(dict(phase=estimate['phase'], stamp=estimate['stamp'],
                    position_error_m=math.hypot(estimate['x']-truth['x'], estimate['y']-truth['y']),
                    yaw_error_rad=abs(math.atan2(math.sin(estimate['yaw']-truth['yaw']),
                                                math.cos(estimate['yaw']-truth['yaw'])))))
        result = dict(profile='nominal zero-noise/zero-slip shared kinematic rover, .06 m radius/.49 m track',
            contracts=contracts, accuracy_verdict='Not established: nominal synthetic consistency only',
            matched_samples=len(matched),
            max_position_error_m=max((s['position_error_m'] for s in matched), default=None),
            max_yaw_error_rad=max((s['yaw_error_rad'] for s in matched), default=None),
            samples=samples, matched_errors=matched)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({key: value for key, value in result.items() if key not in ('samples', 'matched_errors')}))
        if not all(contracts.values()):
            raise RuntimeError('Local synthetic contracts failed; inspect output JSON')
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
