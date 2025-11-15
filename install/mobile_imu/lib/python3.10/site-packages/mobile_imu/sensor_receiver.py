#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from flask import Flask, request
import threading
import json
import time


class SensorReceiver(Node):
    def __init__(self):
        super().__init__('sensor_receiver')
        
        # Publisher for raw IMU data
        self.imu_pub = self.create_publisher(Imu, '/mobile/imu/raw', 10)
        
        # Flask app for receiving HTTP POST requests
        self.app = Flask(__name__)
        self.app.add_url_rule('/sensor', 'sensor', self.sensor_callback, methods=['POST'])
        
        # Data storage
        self.latest_accel = [0.0, 0.0, 0.0]
        self.latest_gyro = [0.0, 0.0, 0.0]
        self.latest_mag = [0.0, 0.0, 0.0]
        self.data_received = False
        
        # Timer to publish at fixed rate
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50 Hz
        
        # Start Flask server in separate thread
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info('Sensor Receiver started on http://0.0.0.0:5000/sensor')
        self.get_logger().info('Waiting for mobile sensor data...')
    
    def run_server(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    
    def sensor_callback(self):
        """Callback for HTTP POST requests from Sensor Logger app"""
        try:
            data = request.get_json()
            
            # Parse accelerometer data
            if 'Acc' in data or 'accelerometer' in data:
                acc_key = 'Acc' if 'Acc' in data else 'accelerometer'
                self.latest_accel = [
                    float(data[acc_key].get('x', 0.0)),
                    float(data[acc_key].get('y', 0.0)),
                    float(data[acc_key].get('z', 0.0))
                ]
            
            # Parse gyroscope data
            if 'Gyr' in data or 'gyroscope' in data:
                gyr_key = 'Gyr' if 'Gyr' in data else 'gyroscope'
                self.latest_gyro = [
                    float(data[gyr_key].get('x', 0.0)),
                    float(data[gyr_key].get('y', 0.0)),
                    float(data[gyr_key].get('z', 0.0))
                ]
            
            # Parse magnetometer data
            if 'Mag' in data or 'magnetometer' in data:
                mag_key = 'Mag' if 'Mag' in data else 'magnetometer'
                self.latest_mag = [
                    float(data[mag_key].get('x', 0.0)),
                    float(data[mag_key].get('y', 0.0)),
                    float(data[mag_key].get('z', 0.0))
                ]
            
            self.data_received = True
            
            return 'OK', 200
            
        except Exception as e:
            self.get_logger().error(f'Error parsing sensor data: {e}')
            return 'Error', 400
    
    def publish_imu(self):
        """Publish IMU data at fixed rate"""
        if not self.data_received:
            return
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mobile_imu_link'
        
        # Accelerometer (m/s^2)
        msg.linear_acceleration.x = self.latest_accel[0]
        msg.linear_acceleration.y = self.latest_accel[1]
        msg.linear_acceleration.z = self.latest_accel[2]
        
        # Gyroscope (rad/s)
        msg.angular_velocity.x = self.latest_gyro[0]
        msg.angular_velocity.y = self.latest_gyro[1]
        msg.angular_velocity.z = self.latest_gyro[2]
        
        # We don't use orientation from phone, so leave it empty
        msg.orientation.w = 1.0
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
