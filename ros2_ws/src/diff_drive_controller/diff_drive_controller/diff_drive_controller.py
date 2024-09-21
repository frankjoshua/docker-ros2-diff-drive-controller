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
        super().__init__('vel_to_odom_publisher')
        
        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'
        
        self.odom_msg_ = Odometry()
        transform_stamped = TransformStamped()
        transform_stamped.header.frame_id = self.frame_id
        transform_stamped.child_frame_id = self.child_frame_id
        self.transform_stamped_ = transform_stamped

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.01
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
        # Send odometry message
        self.publisher_.publish(self.odom_msg_)
        # Send transform
        self.br.sendTransform(self.transform_stamped_)

    def listener_callback(self, msg):
        # Update time
        time_now = self.get_clock().now()
        time_delta = time_now - self.last_update_
        self.last_update_ = time_now
        
        # Update position
        self.updatePosition(msg, time_now, time_delta)
        
        # Update odometry message
        position = self.createPoint(self.position_.x, self.position_.y)
        oriention = self.createQuaternion(self.position_)
        pose = self.createPose(position, oriention)
        twist = self.createTwist(msg)
        self.odom_msg_.header.stamp = time_now.to_msg()
        self.odom_msg_.header.frame_id = self.frame_id
        self.odom_msg_.child_frame_id = self.child_frame_id
        self.odom_msg_.pose.pose = pose
        self.odom_msg_.twist = twist
        
        # Update transform message
        self.transform_stamped_ = self.createTransformStamped(time_now, oriention)

        # self.get_logger().info('I heard: "%s"' % str(self.position_))
        # self.get_logger().info('I heard: "%s"' % str(type(time_now)))
    
    def updatePosition(self, msg, time_delta):
        time_delta_seconds = time_delta.nanoseconds / 1000000000
        dx = msg.linear.x
        dtheta = msg.angular.z
        self.position_.x += math.cos(self.position_.theta) * dx * time_delta_seconds
        self.position_.y += math.sin(self.position_.theta) * dx * time_delta_seconds
        self.position_.theta += dtheta * time_delta_seconds
    
    def createTwist(self, twist_msg):
        twist = TwistWithCovariance()
        twist.twist = twist_msg
    
    def createQuaternion(self, position):
        q = tf_transformations.quaternion_from_euler(0, 0, position.theta)
        oriention = Quaternion()
        oriention.x = q[0]
        oriention.y = q[1]
        oriention.z = q[2]
        oriention.w = q[3]
    
    def createPoint(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        return point
    
    def createPose(self, position, oriention):
        pose = Pose()
        pose.position = position
        pose.orientation = oriention
        return pose
        
    def createTransformStamped(self, time_now, quaternion):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = time_now.to_msg()
        transform_stamped.header.frame_id = self.frame_id
        transform_stamped.child_frame_id = self.child_frame_id
        transform_stamped.transform.translation.x = self.position_.x
        transform_stamped.transform.translation.y = self.position_.y
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]
        return transform_stamped


def main(args=None):
    rclpy.init(args=args)
    
    odom_publisher = OdomPublisher()
    odom_publisher.get_logger().info('diff_drive_controller running.')
    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
