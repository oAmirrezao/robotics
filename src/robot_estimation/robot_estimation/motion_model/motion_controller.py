#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.1)       # meters
        self.declare_parameter('wheel_separation', 0.5)   # meters
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for wheel velocities (rad/s)
        self.left_wheel_pub = self.create_publisher(
            Float64,
            '/left_wheel_velocity_cmd',
            10
        )
        self.right_wheel_pub = self.create_publisher(
            Float64,
            '/right_wheel_velocity_cmd',
            10
        )
        
        self.get_logger().info('Motion Controller node started.')
    
    def cmd_vel_callback(self, msg):
        """
        Convert cmd_vel (linear and angular velocity) to wheel velocities
        
        Differential drive kinematics:
        v_left = (2 * v - omega * L) / (2 * r)
        v_right = (2 * v + omega * L) / (2 * r)
        
        where:
        v = linear velocity (m/s)
        omega = angular velocity (rad/s)
        L = wheel separation (m)
        r = wheel radius (m)
        """
        
        linear_vel = msg.linear.x   # m/s
        angular_vel = msg.angular.z # rad/s
        
        # Calculate wheel velocities (rad/s)
        v_left = (2.0 * linear_vel - angular_vel * self.wheel_separation) / (2.0 * self.wheel_radius)
        v_right = (2.0 * linear_vel + angular_vel * self.wheel_separation) / (2.0 * self.wheel_radius)
        
        # Publish wheel velocities
        left_msg = Float64()
        left_msg.data = v_left
        self.left_wheel_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = v_right
        self.right_wheel_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
