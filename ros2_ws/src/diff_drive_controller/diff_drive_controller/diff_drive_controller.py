import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TransformStamped, TwistWithCovariance
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def __str__(self):
        return f"x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}"


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('vel_to_odom_publisher')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('vel_topic', 'vel')
        self.declare_parameter('enable_logging', False)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.frame_id = self.get_parameter('odom_frame').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.vel_topic = self.get_parameter('vel_topic').value
        self.enable_logging = self.get_parameter('enable_logging').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        rate_hz = max(1.0, min(rate_hz, 200.0))
        self.timer_period = 1.0 / rate_hz

        # State
        self.position_ = Position()
        self.last_update_ = self.get_clock().now()
        self.last_vel_ = Twist()

        # ROS interfaces
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(Twist, self.vel_topic, self.listener_callback, 10)
        self.br = TransformBroadcaster(self)

        # ─── Pre-allocated messages ──────────────────────────────
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = self.frame_id
        self.odom_msg_.child_frame_id = self.child_frame_id
        self.odom_msg_.pose.pose = Pose()
        self.odom_msg_.twist = TwistWithCovariance()

        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = self.frame_id
        self.transform_stamped_.child_frame_id = self.child_frame_id

        # Timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f"Publishing '{self.frame_id}' → '{self.child_frame_id}', "
            f"subscribed to '{self.vel_topic}', rate={rate_hz:.1f} Hz, "
            f"logging={'on' if self.enable_logging else 'off'}"
        )

    # ──────────────────────────────────────────────────────────────────────
    def listener_callback(self, msg):
        self.last_vel_ = msg
        if self.enable_logging:
            self.get_logger().info(
                f"Received velocity: lin.x={msg.linear.x:.3f}, ang.z={msg.angular.z:.3f}"
            )

    # ──────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        time_now = self.get_clock().now()
        dt = (time_now - self.last_update_).nanoseconds / 1e9
        self.last_update_ = time_now
        if dt > 1.0:
            self.get_logger().warn('Large time delta; skipping update.')
            return

        # Integrate motion
        v = self.last_vel_
        self.position_.x += math.cos(self.position_.theta) * v.linear.x * dt
        self.position_.y += math.sin(self.position_.theta) * v.linear.x * dt
        self.position_.theta += v.angular.z * dt

        # Reuse quaternion object
        q = tf_transformations.quaternion_from_euler(0, 0, self.position_.theta)

        # Update pre-allocated odom message in place
        o = self.odom_msg_
        o.header.stamp = time_now.to_msg()
        o.pose.pose.position.x = self.position_.x
        o.pose.pose.position.y = self.position_.y
        o.pose.pose.position.z = 0.0
        o.pose.pose.orientation.x = q[0]
        o.pose.pose.orientation.y = q[1]
        o.pose.pose.orientation.z = q[2]
        o.pose.pose.orientation.w = q[3]
        o.twist.twist = v

        # Update pre-allocated transform in place
        t = self.transform_stamped_
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.position_.x
        t.transform.translation.y = self.position_.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Publish
        self.br.sendTransform(t)
        self.publisher_.publish(o)
        

        if self.enable_logging:
            self.get_logger().info(f"Updated position: {self.position_}")


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    node.get_logger().info('diff_drive_controller running.')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
