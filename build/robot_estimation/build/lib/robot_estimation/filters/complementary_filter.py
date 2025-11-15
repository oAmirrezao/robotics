#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
import numpy as np
import math


class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter')
        
        # Parameters
        self.declare_parameter('alpha', 0.98)  # Complementary filter weight (gyro weight)
        self.alpha = self.get_parameter('alpha').value
        
        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/filtered',
            self.imu_callback,
            10
        )
        
        # Publisher
        self.orientation_pub = self.create_publisher(
            QuaternionStamped,
            '/estimation/orientation',
            10
        )
        
        # State variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        
        self.get_logger().info('Complementary Filter node started.')
    
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:  # Skip invalid dt
            return
        
        # Get gyroscope data (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Get accelerometer data
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        
        # Gyroscope integration (predict)
        gyro_roll = self.roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt
        gyro_yaw = self.yaw + gyro_z * dt
        
        # Accelerometer angles (measure)
        accel_roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        
        # Complementary filter
        # orientation = alpha * gyro + (1 - alpha) * accel
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw  # No accelerometer correction for yaw
        
        # Convert Euler angles to quaternion
        quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        # Publish orientation
        orientation_msg = QuaternionStamped()
        orientation_msg.header = msg.header
        orientation_msg.quaternion.x = quat[0]
        orientation_msg.quaternion.y = quat[1]
        orientation_msg.quaternion.z = quat[2]
        orientation_msg.quaternion.w = quat[3]
        
        self.orientation_pub.publish(orientation_msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion (x, y, z, w)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
