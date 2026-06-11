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

    def __str__(self):
        return f"x={self.x}, y={self.y}, theta={self.theta}"

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('vel_to_odom_publisher')
        
        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'
        
        # Preallocate odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = self.frame_id
        self.odom_msg_.child_frame_id = self.child_frame_id
        
        # Preallocate transform message
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = self.frame_id
        self.transform_stamped_.child_frame_id = self.child_frame_id

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.05  # Reduced update rate (20 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Twist,
            'vel',
            self.listener_callback,
            10)
        self.last_update_ = self.get_clock().now()
        self.position_ = Position()
        self.last_update_ = self.get_clock().now()
        self.last_vel_ = Twist()

        # ROS interfaces
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(Twist, self.vel_topic, self.listener_callback, 10)
        self.br = TransformBroadcaster(self)
        
    def timer_callback(self):
        # Publish preallocated and updated odometry message
        self.publisher_.publish(self.odom_msg_)
        # Broadcast preallocated and updated transform
        self.br.sendTransform(self.transform_stamped_)

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
        time_now = self.get_clock().now()
        time_delta = time_now - self.last_update_
        self.last_update_ = time_now
        
        self.updatePosition(msg, time_delta)
        quaternion = self.createQuaternion(self.position_)

        # Update odometry message
        self.odom_msg_.header.stamp = time_now.to_msg()
        self.odom_msg_.pose.pose.position = self.createPoint(self.position_.x, self.position_.y)
        self.odom_msg_.pose.pose.orientation = quaternion
        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist = msg
        self.odom_msg_.twist = twist_with_cov

        # Update transform message
        self.transform_stamped_.header.stamp = time_now.to_msg()
        self.transform_stamped_.transform.translation.x = self.position_.x
        self.transform_stamped_.transform.translation.y = self.position_.y
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation = quaternion

    def updatePosition(self, msg, time_delta):
        # Convert time delta from nanoseconds to seconds
        time_delta_seconds = time_delta.nanoseconds / 1e9
        
        dx = msg.linear.x
        dtheta = msg.angular.z
        self.position_.x += math.cos(self.position_.theta) * dx * time_delta_seconds
        self.position_.y += math.sin(self.position_.theta) * dx * time_delta_seconds
        self.position_.theta += dtheta * time_delta_seconds

    def createQuaternion(self, position):
        q = tf_transformations.quaternion_from_euler(0, 0, position.theta)
        quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return quaternion

    def createPoint(self, x, y):
        return Point(x=x, y=y, z=0.0)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    odom_publisher.get_logger().info('diff_drive_controller running.')
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
