#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class MobileImuFilter(Node):
    def __init__(self):
        super().__init__('mobile_imu_filter')
        
        # Parameters
        self.declare_parameter('alpha_accel', 0.2)
        self.declare_parameter('alpha_gyro', 0.2)
        self.declare_parameter('bias_samples', 200)
        
        self.alpha_accel = self.get_parameter('alpha_accel').value
        self.alpha_gyro = self.get_parameter('alpha_gyro').value
        self.bias_samples = self.get_parameter('bias_samples').value
        
        # Subscriber and Publisher
        self.imu_sub = self.create_subscription(
            Imu,
            '/mobile/imu/raw',
            self.imu_callback,
            10
        )
        
        self.imu_filtered_pub = self.create_publisher(
            Imu,
            '/mobile/imu/filtered',
            10
        )
        
        # State variables
        self.accel_filtered = np.array([0.0, 0.0, 0.0])
        self.gyro_filtered = np.array([0.0, 0.0, 0.0])
        
        # Bias estimation
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, -9.81])  # Phone usually has gravity on -z
        self.bias_samples_collected = 0
        self.bias_sum_gyro = np.array([0.0, 0.0, 0.0])
        self.bias_sum_accel = np.array([0.0, 0.0, 0.0])
        self.bias_calibrated = False
        
        self.get_logger().info('Mobile IMU Filter started. Place phone flat and still for calibration...')
    
    def imu_callback(self, msg):
        # Extract data
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Bias calibration phase
        if not self.bias_calibrated:
            if self.bias_samples_collected < self.bias_samples:
                self.bias_sum_gyro += gyro
                self.bias_sum_accel += accel
                self.bias_samples_collected += 1
                
                if self.bias_samples_collected % 50 == 0:
                    self.get_logger().info(f'Calibrating... {self.bias_samples_collected}/{self.bias_samples}')
                return
            else:
                self.gyro_bias = self.bias_sum_gyro / self.bias_samples
                self.accel_bias = self.bias_sum_accel / self.bias_samples
                
                # Determine gravity direction
                accel_mag = np.linalg.norm(self.accel_bias)
                self.accel_bias = self.accel_bias - (self.accel_bias / accel_mag) * 9.81
                
                self.bias_calibrated = True
                self.get_logger().info('=' * 50)
                self.get_logger().info('Bias calibration complete!')
                self.get_logger().info(f'Gyro bias: {self.gyro_bias}')
                self.get_logger().info(f'Accel bias: {self.accel_bias}')
                self.get_logger().info('=' * 50)
        
        # Remove bias
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias
        
        # Low-pass filter
        self.accel_filtered = (self.alpha_accel * accel_corrected + 
                              (1 - self.alpha_accel) * self.accel_filtered)
        
        self.gyro_filtered = (self.alpha_gyro * gyro_corrected + 
                             (1 - self.alpha_gyro) * self.gyro_filtered)
        
        # Publish filtered IMU
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        filtered_msg.linear_acceleration.x = self.accel_filtered[0]
        filtered_msg.linear_acceleration.y = self.accel_filtered[1]
        filtered_msg.linear_acceleration.z = self.accel_filtered[2]
        
        filtered_msg.angular_velocity.x = self.gyro_filtered[0]
        filtered_msg.angular_velocity.y = self.gyro_filtered[1]
        filtered_msg.angular_velocity.z = self.gyro_filtered[2]
        
        filtered_msg.orientation = msg.orientation
        
        self.imu_filtered_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MobileImuFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
