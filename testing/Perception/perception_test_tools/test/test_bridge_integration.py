import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3D,Detection3DArray
from sensor_msgs.msg import PointCloud2


def test_bridge_empty_invalid_and_bounded_geometry():
    rclpy.init();node=Node('bridge_regression_observer');received=[]
    publisher=node.create_publisher(Detection3DArray,'/testing/bridge/boxes',10)
    subscription=node.create_subscription(PointCloud2,'/testing/bridge/cloud',received.append,10)
    with open('/tmp/perception-bridge.log','w') as log:
        process=subprocess.Popen(['ros2','run','erc_path_planner','costmap_bridge_node','--ros-args',
            '-p','input_topic:=/testing/bridge/boxes','-p','output_topic:=/testing/bridge/cloud','-p','max_points:=100'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            def observe(message):
                received.clear();deadline=time.monotonic()+8
                while time.monotonic()<deadline and process.poll() is None:
                    publisher.publish(message);rclpy.spin_once(node,timeout_sec=.1)
                    matches=[value for value in received if value.header.stamp.sec==message.header.stamp.sec]
                    if matches:return matches[-1]
                raise AssertionError('No bridge response; inspect /tmp/perception-bridge.log')
            message=Detection3DArray();message.header.frame_id='test_frame';message.header.stamp.sec=101
            assert observe(message).width==0
            box=Detection3D();box.bbox.center.position.x=float('nan')
            box.bbox.size.x=box.bbox.size.y=box.bbox.size.z=1.;box.bbox.center.orientation.w=1.
            message.detections=[box];message.header.stamp.sec=102
            assert observe(message).width==0
            box.bbox.center.position.x=2.;box.bbox.size.x=box.bbox.size.y=10.
            message.detections=[box];message.header.stamp.sec=103
            cloud=observe(message)
            assert 0<cloud.width<=100
            assert cloud.header.frame_id=='test_frame'
            box.bbox.center.orientation.w=1e308
            message.detections=[box];message.header.stamp.sec=104
            assert observe(message).width==0
            box.bbox.center.orientation.w=box.bbox.center.orientation.x=2**-.5
            box.bbox.size.x=box.bbox.size.y=.2;box.bbox.size.z=2.
            message.detections=[box];message.header.stamp.sec=105
            from sensor_msgs_py import point_cloud2
            cloud=observe(message)
            assert max(float(row[1]) for row in point_cloud2.read_points(cloud,field_names=('x','y','z'))) > .8
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=10);node.destroy_node();rclpy.shutdown()
