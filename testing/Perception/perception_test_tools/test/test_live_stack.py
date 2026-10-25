"""Real headless ROS pipeline, isolated domain supplied by the test runner."""
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from marker_detection_msgs.msg import MarkerTrackedPoseArray
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid


def test_real_pipelines_process_synthetic_sensors():
    rclpy.init()
    observer=Node('perception_integration_observer')
    received={'boxes':[],'poses':[],'grids':[],'health':[]}
    subscriptions=[
        observer.create_subscription(Detection3DArray,'/perception/local_bboxes',received['boxes'].append,10),
        observer.create_subscription(MarkerTrackedPoseArray,'/marker_poses',received['poses'].append,10),
        observer.create_subscription(OccupancyGrid,'/terrain/costmap',received['grids'].append,10),
        observer.create_subscription(DiagnosticArray,'/terrain/diagnostics',received['health'].append,10)]
    with open('/tmp/perception-live-stack.log','w') as log:
        process=subprocess.Popen(['ros2','launch','perception_test_tools','test_stack.launch.py','launch_rviz:=false'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+35
            while time.monotonic()<deadline and process.poll() is None:
                rclpy.spin_once(observer,timeout_sec=.1)
                if (any(m.detections for m in received['boxes']) and
                    any(p.id==0 and p.depth_fused for m in received['poses'] for p in m.markers) and
                    any(any(v>0 for v in m.data) for m in received['grids']) and
                    any(s.level in (0,bytes([0])) for m in received['health'] for s in m.status)):
                    break
            assert any(m.detections for m in received['boxes']), 'No rock observations; inspect /tmp/perception-live-stack.log'
            assert any(p.id==0 and p.depth_fused for m in received['poses'] for p in m.markers), 'No depth-fused marker'
            assert any(any(v>0 for v in m.data) for m in received['grids']), 'No occupied costmap cells'
            assert any(s.level in (0,bytes([0])) for m in received['health'] for s in m.status), 'Terrain never healthy'
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            observer.destroy_node();rclpy.shutdown()
