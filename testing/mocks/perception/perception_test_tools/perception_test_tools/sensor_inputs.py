"""Synthetic publishers: installed only by the explicit testing package."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo,Image,PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster
from visualization_msgs.msg import Marker,MarkerArray
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from .scene import SCENARIOS,OPTICAL_TO_BASE,MOUNT,make_scene


class SensorInputs(Node):
    def __init__(self):
        super().__init__('perception_test_inputs')
        for key,value in {'scenario':'rocks_and_marker','seed':0,'rate':10.,'synthetic_motion':False}.items():
            self.declare_parameter(key,value)
        if self.get_parameter('scenario').value not in SCENARIOS or self.get_parameter('rate').value<=0:
            raise ValueError('Valid scenario and positive rate required')
        prefix='/testing/perception/camera/'
        self.rgb=self.create_publisher(Image,prefix+'image_raw',qos_profile_sensor_data)
        self.depth=self.create_publisher(Image,prefix+'depth',qos_profile_sensor_data)
        self.info=self.create_publisher(CameraInfo,prefix+'image_info',qos_profile_sensor_data)
        self.points=self.create_publisher(PointCloud2,prefix+'points',qos_profile_sensor_data)
        self.truth=self.create_publisher(MarkerArray,'/testing/perception/ground_truth',10)
        self.static=StaticTransformBroadcaster(self)
        self.dynamic=TransformBroadcaster(self)
        tf=TransformStamped()
        tf.header.frame_id='base_link';tf.child_frame_id='test_camera_optical_frame'
        tf.header.stamp=self.get_clock().now().to_msg()
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=MOUNT
        q=Rotation.from_matrix(OPTICAL_TO_BASE).as_quat()
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=q
        self.static.sendTransform(tf)
        self.bridge=CvBridge();self.tick=0;self.pending_rgb=None
        self.create_timer(.02,self.publish_rgb)
        self.create_timer(1/self.get_parameter('rate').value,self.publish)

    def publish_rgb(self):
        # Give independent depth/calibration subscribers time to buffer this
        # stamp before triggering the real detector's RGB-driven callback.
        if self.pending_rgb is not None:
            self.rgb.publish(self.pending_rgb)
            self.pending_rgb=None

    def publish(self):
        self.tick+=1
        scenario=self.get_parameter('scenario').value
        if scenario=='dropout' and self.tick%40>=20:
            return
        stamp=self.get_clock().now().to_msg()
        header=Header(stamp=stamp,frame_id='test_camera_optical_frame')
        points,rgb,depth,visible=make_scene(scenario,self.get_parameter('seed').value,self.tick)
        if self.get_parameter('synthetic_motion').value:
            # Known test yaw. Rotate world-fixed geometry into the moving base.
            yaw=.02*self.tick
            base=points@OPTICAL_TO_BASE.T+MOUNT
            base=Rotation.from_euler('z',-yaw).apply(base)
            points=((base-MOUNT)@OPTICAL_TO_BASE).astype(np.float32)
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp
            q=Rotation.from_euler('z',yaw).as_quat()
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=q
            self.dynamic.sendTransform(tf)
        self.points.publish(point_cloud2.create_cloud_xyz32(header,points))
        image=self.bridge.cv2_to_imgmsg(rgb,encoding='bgr8');image.header=header;self.pending_rgb=image
        if scenario!='missing_depth':
            image=self.bridge.cv2_to_imgmsg(depth,encoding='32FC1');image.header=header;self.depth.publish(image)
        info=CameraInfo();info.header=header;info.width=640;info.height=480
        info.k=[600.,0.,320.,0.,600.,240.,0.,0.,1.]
        info.p=[600.,0.,320.,0.,0.,600.,240.,0.,0.,0.,1.,0.]
        info.distortion_model='plumb_bob';info.d=[0.]*5
        self.info.publish(info)
        output=MarkerArray()
        rock=Marker();rock.header=Header(stamp=stamp,frame_id='odom' if self.get_parameter('synthetic_motion').value else 'base_link')
        rock.ns='expected_geometry';rock.id=0;rock.type=Marker.CUBE
        rock.action=Marker.ADD if visible else Marker.DELETE
        rock.pose.position.x=2.;rock.pose.position.y=.6;rock.pose.position.z=.25;rock.pose.orientation.w=1.
        rock.scale.x=rock.scale.y=rock.scale.z=.4;rock.color.g=1.;rock.color.a=.35
        output.markers=[rock];self.truth.publish(output)


def main(args=None):
    rclpy.init(args=args);node=SensorInputs()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
