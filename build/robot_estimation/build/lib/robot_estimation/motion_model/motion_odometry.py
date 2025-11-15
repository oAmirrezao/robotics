#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
import math


class MotionOdometry(Node):
    def __init__(self):
        super().__init__('motion_odometry')
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.1)       # meters
        self.declare_parameter('wheel_separation', 0.5)   # meters
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Subscribers for wheel RPM
        self.left_rpm_sub = self.create_subscription(
            Float64,
            '/left_wheel_rpm',
            self.left_rpm_callback,
            10
        )
        self.right_rpm_sub = self.create_subscription(
            Float64,
            '/right_wheel_rpm',
            self.right_rpm_callback,
            10
        )
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/motion_model/odom',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        
        self.last_time = self.get_clock().now()
        
        # Timer for odometry calculation
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20 Hz
        
        self.get_logger().info('Motion Odometry node started.')
    
    def left_rpm_callback(self, msg):
        self.left_rpm = msg.data
    
    def right_rpm_callback(self, msg):
        self.right_rpm = msg.data
    
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0 or dt > 1.0:
            self.last_time = current_time
            return
        
        # Convert RPM to rad/s
        left_vel = (self.left_rpm * 2.0 * math.pi) / 60.0   # rad/s
        right_vel = (self.right_rpm * 2.0 * math.pi) / 60.0 # rad/s
        
        # Calculate linear and angular velocities
        # v = r * (v_right + v_left) / 2
        # omega = r * (v_right - v_left) / L
        v = self.wheel_radius * (right_vel + left_vel) / 2.0
        omega = self.wheel_radius * (right_vel - left_vel) / self.wheel_separation
        
        # Update pose using differential drive kinematics
        delta_theta = omega * dt
        delta_x = v * math.cos(self.theta + delta_theta / 2.0) * dt
        delta_y = v * math.sin(self.theta + delta_theta / 2.0) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        quat = self.yaw_to_quaternion(self.theta)
        
        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)
        
        self.last_time = current_time
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (x, y, z, w)"""
        return [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        ]


def main(args=None):
    rclpy.init(args=args)
    node = MotionOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
