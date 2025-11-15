#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
import math


class PositionEstimator(Node):
    def __init__(self):
        super().__init__('position_estimator')
        
        # Parameters
        self.declare_parameter('alpha', 0.98)
        self.alpha = self.get_parameter('alpha').value
        
        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/mobile/imu/filtered',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/mobile/path', 10)
        
        # State variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        self.last_time = None
        
        self.get_logger().info('Position Estimator started.')
        self.get_logger().info('Move the phone in a square path!')
    
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0:
            return
        
        # Get sensor data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        # === Complementary Filter for Orientation ===
        
        # Gyroscope integration
        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt
        gyro_yaw = self.yaw + gz * dt
        
        # Accelerometer angles
        accel_roll = math.atan2(ay, math.sqrt(ax**2 + az**2))
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # Complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw
        
        # === Position Estimation ===
        
        # Remove gravity from acceleration (rotate to world frame)
        cos_roll = math.cos(self.roll)
        sin_roll = math.sin(self.roll)
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Rotation matrix from body to world frame
        # Simplified for small angles
        ax_world = (ax * cos_pitch * cos_yaw + 
                   ay * (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw) +
                   az * (cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw))
        
        ay_world = (ax * cos_pitch * sin_yaw + 
                   ay * (sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw) +
                   az * (cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw))
        
        az_world = (-ax * sin_pitch + 
                   ay * sin_roll * cos_pitch +
                   az * cos_roll * cos_pitch)
        
        # Remove gravity (assuming z points up)
        az_world -= 9.81
        
        # Integration for velocity
        self.vx += ax_world * dt
        self.vy += ay_world * dt
        self.vz += az_world * dt
        
        # Apply velocity damping to reduce drift
        damping = 0.95
        self.vx *= damping
        self.vy *= damping
        self.vz *= damping
        
        # Integration for position
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        
        # Create pose for path
        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = 'map'
        
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        
        # Orientation as quaternion
        quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        # Add to path
        self.path.poses.append(pose)
        
        # Keep only last 1000 poses
        if len(self.path.poses) > 1000:
            self.path.poses = self.path.poses[-1000:]
        
        # Publish path
        self.path.header.stamp = current_time.to_msg()
        self.path_pub.publish(self.path)
        
        # Log position every 2 seconds
        if len(self.path.poses) % 100 == 0:
            self.get_logger().info(f'Position: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
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
    node = PositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
