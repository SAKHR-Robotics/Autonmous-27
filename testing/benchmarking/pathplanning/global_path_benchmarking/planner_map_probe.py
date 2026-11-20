"""Reproduce boundary/clearance gates against independently generated maps.

Run against a disposable mocked_planner launch: this changes its map_server map.
A successful action whose path collides with scenario truth fails validation.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
import yaml

from global_path_benchmarking.navigation_probe import swept_collision, yaw


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    options = parser.parse_args(args)
    root = Path(options.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('independent_map_scenarios')
    records = []
    load = node.create_client(LoadMap, '/map_server/load_map')
    clear = node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')

    def wait(future, seconds=15):
        rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
        if not future.done():
            raise TimeoutError('Service/action response unavailable')
        return future.result()

    def pose(x, y):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = float(x), float(y)
        message.pose.orientation.w = 1.0
        return message

    try:
        ready = (load.wait_for_service(timeout_sec=15)
                 and clear.wait_for_service(timeout_sec=15)
                 and planner.wait_for_server(timeout_sec=30))
        for label, gap in [('blocked-wall', 0.0), ('too-narrow', 0.6), ('wide-passage', 2.4)]:
            record = {'scenario': label, 'validation_pass': False, 'status': None,
                      'swept_collision': None, 'outcome': 'INCOMPLETE'}
            try:
                if not ready:
                    raise RuntimeError('Required map/costmap services or planner action unavailable')
                image = np.full((240, 240), 255, dtype=np.uint8)
                image[:, 119:122] = 0
                if gap:
                    center, half = 60, int(gap / 0.05 / 2)
                    image[240-center-half:240-center+half, 119:122] = 255
                Image.fromarray(image).save(root / (label + '.pgm'))
                spec = {'image': label + '.pgm', 'resolution': 0.05,
                        'origin': [-6.0, -6.0, 0.0], 'negate': 0,
                        'free_thresh': 0.196, 'occupied_thresh': 0.65}
                map_file = root / (label + '.yaml')
                map_file.write_text(yaml.safe_dump(spec))
                request = LoadMap.Request()
                request.map_url = str(map_file)
                if wait(load.call_async(request)).result != 0:
                    raise RuntimeError('Scenario map failed to load')
                wait(clear.call_async(ClearEntireCostmap.Request()))
                time.sleep(2)  # Allow the configured 1 Hz global costmap to update.
                goal = ComputePathToPose.Goal()
                goal.start, goal.goal = pose(-3, -3), pose(3, -3)
                goal.use_start, goal.planner_id = True, 'GridBased'
                started = time.monotonic()
                handle = wait(planner.send_goal_async(goal))
                if not handle.accepted:
                    raise RuntimeError('Planner rejected the request')
                result = wait(handle.get_result_async(), 20)
                elapsed = time.monotonic() - started
                path = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                        for p in result.result.path.poses]
                record.update(goal_id=bytes(handle.goal_id.uuid).hex(), status=result.status,
                              error_code=result.result.error_code, latency_sec=elapsed,
                              swept_collision=swept_collision(path, str(map_file)), path=path)
                record['validation_pass'] = (
                    result.status == 4 and record['swept_collision'] is False
                    if label == 'wide-passage' else result.status == 6 and not path)
                record['outcome'] = 'PASS' if record['validation_pass'] else 'FAIL'
            except Exception as error:
                record['error'] = str(error)
            records.append(record)
            (root / 'map-scenarios.json').write_text(json.dumps(records, indent=2) + '\n')
            print(json.dumps({k: v for k, v in record.items() if k != 'path'}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if all(item['validation_pass'] for item in records) else 1


if __name__ == '__main__':
    raise SystemExit(main())
