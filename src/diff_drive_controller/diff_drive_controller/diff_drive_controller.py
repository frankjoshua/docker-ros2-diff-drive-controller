import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TransformStamped, TwistWithCovariance
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster

import math

class Position():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def  __str__(self):
        return "x={0},y={1},theta={2}".format(self.x, self.y, self.theta)

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_msg_ = Odometry()
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Twist,
            'vel',
            self.listener_callback,
            10)
        self.last_update_ = self.get_clock().now()
        self.position_ = Position()
        self.br = TransformBroadcaster(self)
        

    def timer_callback(self):
        self.publisher_.publish(self.odom_msg_)

    def listener_callback(self, msg):
        time_now = self.get_clock().now()
        time_delta = time_now - self.last_update_
        self.last_update_ = time_now
        time_delta_seconds = time_delta.nanoseconds / 1000000000
        dx = msg.linear.x
        dtheta = msg.angular.z
        self.position_.x += math.cos(self.position_.theta) * dx * time_delta_seconds
        self.position_.y += math.sin(self.position_.theta) * dx * time_delta_seconds
        self.position_.theta += dtheta * time_delta_seconds

        self.odom_msg_.header.stamp = time_now.to_msg()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        q = tf_transformations.quaternion_from_euler(0, 0, self.position_.theta)
        position = Point()
        position.x = self.position_.x
        position.y = self.position_.y
        position.z = 0.0
        oriention = Quaternion()
        oriention.x = q[0]
        oriention.y = q[1]
        oriention.z = q[2]
        oriention.w = q[3]
        pose = Pose()
        pose.position = position
        pose.orientation = oriention
        self.odom_msg_.pose.pose = pose
        twist = TwistWithCovariance()
        twist.twist = msg
        self.odom_msg_.twist = twist

        t = TransformStamped()
        t.header.stamp = time_now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.position_.x
        t.transform.translation.y = self.position_.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        # self.get_logger().info('I heard: "%s"' % str(self.position_))
        # self.get_logger().info('I heard: "%s"' % str(type(time_now)))
        


def main(args=None):
    rclpy.init(args=args)
    
    odom_publisher = OdomPublisher()
    odom_publisher.get_logger().info('diff_drive_controller running.')
    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
