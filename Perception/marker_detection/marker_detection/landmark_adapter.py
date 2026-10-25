#!/usr/bin/env python3
"""Quality-gated tracked observations for RTAB-Map's actual landmark input."""
import numpy as np
from marker_detection.action_target_builder import _DISQUALIFYING_FLAGS


def landmark_from_track(track, *, now, max_age=.5, min_confidence=.6, id_offset=1,
                        angular_variance=.1, position_variance=.04):
    from rtabmap_msgs.msg import LandmarkDetection
    stamp=track.header.stamp.sec+track.header.stamp.nanosec*1e-9
    identifier=int(track.id)+id_offset
    if not (track.detection_valid and track.tracking_valid and track.track_state=='CONFIRMED'
            and track.tracking_confidence >= min_confidence and track.age_seconds <= max_age
            and 0 <= now-stamp <= max_age and 0 < identifier < 2**31
            and not (int(track.quality_flags) & int(_DISQUALIFYING_FLAGS))):
        return None
    position=np.array([track.position.x,track.position.y,track.position.z])
    orientation=np.array([track.orientation.x,track.orientation.y,track.orientation.z,track.orientation.w])
    covariance=np.array(track.position_covariance).reshape(3,3)
    if not np.isfinite(position).all() or not np.isfinite(orientation).all() or np.linalg.norm(orientation)<1e-8:
        return None
    if not np.isfinite(covariance).all() or not np.allclose(covariance,covariance.T) or np.min(np.linalg.eigvalsh(covariance)) < -1e-9:
        return None
    if not np.any(covariance):
        covariance=np.eye(3)*position_variance
    message=LandmarkDetection()
    message.header=track.header
    message.id=identifier
    message.landmark_frame_id=f'aruco_marker_{track.id}'
    message.pose.pose.position=track.position
    q=orientation/np.linalg.norm(orientation)
    message.pose.pose.orientation.x,message.pose.pose.orientation.y,message.pose.pose.orientation.z,message.pose.pose.orientation.w=q
    full=np.zeros((6,6)); full[:3,:3]=covariance; full[3:,3:]=np.eye(3)*angular_variance
    message.pose.covariance=full.ravel().tolist()
    return message


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from marker_detection_msgs.msg import MarkerTrackedPoseArray
    from rtabmap_msgs.msg import LandmarkDetections

    class Adapter(Node):
        def __init__(self):
            super().__init__('marker_landmark_adapter')
            for key,value in {'max_age':.5,'min_confidence':.6,'id_offset':1,'angular_variance':.1}.items():
                self.declare_parameter(key,value)
            if self.get_parameter('max_age').value <= 0 or self.get_parameter('angular_variance').value <= 0:
                raise ValueError('Positive landmark age and angular variance required')
            self.pub=self.create_publisher(LandmarkDetections,'/perception/landmark_detections',10)
            self.create_subscription(MarkerTrackedPoseArray,'/marker_poses',self.observe,10)
        def observe(self,msg):
            output=LandmarkDetections()
            parameters={key:self.get_parameter(key).value for key in ('max_age','min_confidence','id_offset','angular_variance')}
            for track in msg.markers:
                landmark=landmark_from_track(track,now=self.get_clock().now().nanoseconds*1e-9,**parameters)
                if landmark is not None:
                    output.landmarks.append(landmark)
            self.pub.publish(output)

    rclpy.init(args=args)
    node=Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
