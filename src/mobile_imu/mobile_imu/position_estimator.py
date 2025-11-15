#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


class PositionEstimator(Node):
    def __init__(self):
        super().__init__('position_estimator')
        
        # Parameters
        self.declare_parameter('alpha', 0.98)
        self.declare_parameter('velocity_damping', 0.98)
        self.declare_parameter('accel_threshold', 0.05)
        
        self.alpha = self.get_parameter('alpha').value
        self.velocity_damping = self.get_parameter('velocity_damping').value
        self.accel_threshold = self.get_parameter('accel_threshold').value
        
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
        
        self.x = 0.0
        self.y = 0.0
        
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        self.last_time = None
        self.sample_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('Position Estimator started.')
        self.get_logger().info('IMPORTANT: Hold phone VERTICAL (like taking a photo)')
        self.get_logger().info('Screen facing you, move FORWARD slowly')
        self.get_logger().info('='*60)
    
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
        
        self.sample_count += 1
        
        # Get sensor data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        # === Orientation Estimation (Only Yaw matters for 2D) ===
        self.yaw += gz * dt
        
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # === Simplified 2D Motion Model ===
        # Assume phone is held vertical (screen facing forward)
        # ax = forward/backward acceleration
        # ay = left/right acceleration
        # az = up/down (should be ~9.81 when still)
        
        # Apply acceleration threshold (noise reduction)
        if abs(ax) < self.accel_threshold:
            ax = 0.0
        if abs(ay) < self.accel_threshold:
            ay = 0.0
        
        # Transform acceleration to world frame using yaw
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        ax_world = ax * cos_yaw - ay * sin_yaw
        ay_world = ax * sin_yaw + ay * cos_yaw
        
        # Update velocity (with damping to reduce drift)
        self.vx = self.vx * self.velocity_damping + ax_world * dt
        self.vy = self.vy * self.velocity_damping + ay_world * dt
        
        # Additional damping for very small velocities
        if abs(self.vx) < 0.01:
            self.vx = 0.0
        if abs(self.vy) < 0.01:
            self.vy = 0.0
        
        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Create and publish pose
        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = 'map'
        
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        
        # Orientation as quaternion (only yaw)
        quat = self.yaw_to_quaternion(self.yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        # Add to path
        self.path.poses.append(pose)
        
        # Keep only last 2000 poses
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]
        
        # Publish path
        self.path.header.stamp = current_time.to_msg()
        self.path_pub.publish(self.path)
        
        # Log every 2 seconds (100 samples at 50Hz)
        if self.sample_count % 100 == 0:
            self.get_logger().info(
                f'Pos: x={self.x:.2f}m, y={self.y:.2f}m | '
                f'Vel: vx={self.vx:.2f}, vy={self.vy:.2f} | '
                f'Yaw: {math.degrees(self.yaw):.1f}° | '
                f'Accel: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}'
            )
    
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
    node = PositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
