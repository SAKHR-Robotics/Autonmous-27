"""Bounded odom-frame obstacle memory; no synthetic localization."""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation


def transform_detection(detection, transform, frame, stamp):
    result = deepcopy(detection)
    translation, quaternion = transform.transform.translation, transform.transform.rotation
    rotation = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    center = result.bbox.center.position
    center.x, center.y, center.z = rotation.apply([center.x, center.y, center.z]) + [translation.x, translation.y, translation.z]
    q = result.bbox.center.orientation
    q.x, q.y, q.z, q.w = (rotation * Rotation.from_quat([q.x, q.y, q.z, q.w])).as_quat()
    result.header.frame_id, result.header.stamp = frame, stamp
    return result


class ObstacleMemory:
    def __init__(self, ttl=5.0, association_distance=0.5, capacity=1000):
        if ttl <= 0 or association_distance <= 0 or capacity < 1:
            raise ValueError("Memory limits must be positive")
        self.ttl, self.distance, self.capacity = ttl, association_distance, capacity
        self.entries = []
        self.last_time = None

    def snapshot(self, now):
        if self.last_time is not None and now < self.last_time:
            self.entries.clear()
        self.last_time = now
        self.entries = [(box, seen) for box, seen in self.entries if 0 <= now - seen <= self.ttl]
        return [deepcopy(box) for box, _ in self.entries]

    def update(self, boxes, now):
        self.snapshot(now)
        used = set()
        for box in boxes:
            c, size = box.bbox.center.position, box.bbox.size
            if not np.isfinite([c.x,c.y,c.z,size.x,size.y,size.z]).all() or min(size.x,size.y,size.z) <= 0:
                continue
            candidates = [(np.linalg.norm(np.array([c.x-old.bbox.center.position.x,
                c.y-old.bbox.center.position.y,c.z-old.bbox.center.position.z])), index)
                for index,(old,_) in enumerate(self.entries) if index not in used]
            match = min(candidates) if candidates else (float('inf'), -1)
            if match[0] <= self.distance:
                self.entries[match[1]] = (deepcopy(box),now)
                used.add(match[1])
            elif len(self.entries) < self.capacity:
                self.entries.append((deepcopy(box),now))
                used.add(len(self.entries)-1)

    def clear_observed_ground(self, points, min_returns=3):
        # Only measured ground inside the inner footprint can remove memory.
        points = np.asarray(points)
        keep = []
        for box,seen in self.entries:
            c,s = box.bbox.center.position,box.bbox.size
            inside = ((abs(points[:,0]-c.x) < s.x*.25) & (abs(points[:,1]-c.y) < s.y*.25)
                      & (points[:,2] <= c.z-s.z*.25))
            if np.count_nonzero(inside) < min_returns:
                keep.append((box,seen))
        self.entries = keep


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer, TransformListener, TransformException
    from vision_msgs.msg import Detection3DArray
    from sensor_msgs.msg import PointCloud2
    from terrain_geometry.tf_transform import pointcloud2_to_xyz_array

    class MemoryNode(Node):
        def __init__(self):
            super().__init__('obstacle_memory')
            for key,value in {'memory_frame':'odom','ttl_sec':5.0,'association_distance':0.5,
                'clear_observed_ground':False,'input_timeout_sec':1.0}.items():
                self.declare_parameter(key,value)
            self.frame = self.get_parameter('memory_frame').value
            self.memory = ObstacleMemory(self.get_parameter('ttl_sec').value,
                self.get_parameter('association_distance').value)
            self.buffer=Buffer()
            self.listener=TransformListener(self.buffer,self)
            self.publisher=self.create_publisher(Detection3DArray,'/perception/obstacles_only',10)
            self.create_subscription(Detection3DArray,'/perception/local_bboxes',self.observe,10)
            self.create_subscription(PointCloud2,'/terrain/debug/ground_cloud',self.ground,qos_profile_sensor_data)
            self.create_timer(.2,self.publish)

        def resolve(self,header):
            return self.buffer.lookup_transform(self.frame,header.frame_id,Time.from_msg(header.stamp))

        def observe(self,msg):
            now=self.get_clock().now().nanoseconds*1e-9
            observed=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if not 0 <= now-observed <= self.get_parameter('input_timeout_sec').value:
                return
            try:
                transform=self.resolve(msg.header)
                boxes=[transform_detection(box,transform,self.frame,msg.header.stamp) for box in msg.detections]
                self.memory.update(boxes,now)
            except (TransformException,ValueError) as error:
                self.get_logger().warning(f'Memory observation rejected: {error}',throttle_duration_sec=5.)

        def ground(self,msg):
            if not self.get_parameter('clear_observed_ground').value:
                return
            now=self.get_clock().now().nanoseconds*1e-9
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if not 0 <= now-stamp <= self.get_parameter('input_timeout_sec').value:
                return
            try:
                transform=self.resolve(msg.header).transform
                q,t=transform.rotation,transform.translation
                points=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(pointcloud2_to_xyz_array(msg))+[t.x,t.y,t.z]
                self.memory.clear_observed_ground(points)
            except (TransformException,ValueError):
                return

        def publish(self):
            now=self.get_clock().now()
            msg=Detection3DArray()
            msg.header.frame_id,self_stamp=self.frame,now.to_msg()
            msg.header.stamp=self_stamp
            msg.detections=self.memory.snapshot(now.nanoseconds*1e-9)
            self.publisher.publish(msg)

    rclpy.init(args=args)
    node=MemoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
