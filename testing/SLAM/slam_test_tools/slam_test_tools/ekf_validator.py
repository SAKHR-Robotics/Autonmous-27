"""Test-only EKF cadence, finite-value and dropout monitor; not a safety interlock."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class EKFValidatorNode(Node):
    def __init__(self):
        super().__init__('ekf_validator')
        defaults={'odom_topic':'/odometry/filtered','expected_frequency_hz':30.,
                  'max_gap_seconds':.25,'max_linear_speed_mps':3.}
        for key,val in defaults.items(): self.declare_parameter(key,val)
        self.settings={key:self.get_parameter(key).value for key in defaults}
        if not all(math.isfinite(v) and v>0 for k,v in self.settings.items() if k!='odom_topic'):
            raise ValueError('Positive finite validation limits required')
        self.last_stamp=self.last_receipt=None
        self.last_valid_status=False
        self.reason='waiting for odometry'
        self.last_clock=self.get_clock().now().nanoseconds/1e9
        self.sub=self.create_subscription(Odometry,self.settings['odom_topic'],self.observe,10)
        self.pub_status=self.create_publisher(Bool,'/ekf/validation_status',10)
        self.pub_frequency=self.create_publisher(Float64,'/ekf/validation_frequency_hz',10)
        self.timer=self.create_timer(.1,self.watchdog)

    def now(self):
        value=self.get_clock().now().nanoseconds/1e9
        if value<self.last_clock:
            self.last_stamp=self.last_receipt=None
            self.last_valid_status=False
            self.reason='clock reset; awaiting new data'
        self.last_clock=value
        return value

    def observe(self,msg):
        now=self.now()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        t,p,q=msg.twist.twist,msg.pose.pose.position,msg.pose.pose.orientation
        values=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z,
                p.x,p.y,p.z,q.x,q.y,q.z,q.w,*msg.pose.covariance,*msg.twist.covariance)
        valid=(all(math.isfinite(v) for v in values) and math.hypot(q.x,q.y,q.z,q.w)>1e-8
               and 0<=now-stamp<=self.settings['max_gap_seconds'])
        if self.last_stamp is not None:
            dt=stamp-self.last_stamp
            valid=valid and dt>0 and dt<=self.settings['max_gap_seconds']
            if dt>0:
                frequency=1./dt
                self.pub_frequency.publish(Float64(data=frequency))
                expected=self.settings['expected_frequency_hz']
                valid=valid and abs(frequency-expected)<=expected*.35
        else:
            valid=False  # A single sample does not establish cadence.
        valid=valid and math.hypot(t.linear.x,t.linear.y,t.linear.z)<=self.settings['max_linear_speed_mps']
        self.last_valid_status=valid
        self.reason='healthy' if valid else 'invalid, stale, irregular or unrealistic odometry'
        if self.last_stamp is None or stamp>self.last_stamp: self.last_stamp=stamp
        self.last_receipt=now
        self.pub_status.publish(Bool(data=valid))

    def watchdog(self):
        now=self.now()
        if (self.last_receipt is None or now-self.last_receipt>self.settings['max_gap_seconds']
                or self.last_stamp is None or now-self.last_stamp>self.settings['max_gap_seconds']):
            self.last_valid_status=False
            self.reason='odometry unavailable or stale'
        self.pub_status.publish(Bool(data=self.last_valid_status))
        if not self.last_valid_status:
            self.get_logger().warning(self.reason,throttle_duration_sec=3.)


def main(args=None):
    rclpy.init(args=args)
    node=EKFValidatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__': main()
