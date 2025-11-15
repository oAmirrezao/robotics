#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math


class WheelRpmPublisher(Node):
    def __init__(self):
        super().__init__('wheel_rpm_publisher')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.1)  # meters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.left_rpm_pub = self.create_publisher(Float64, '/left_wheel_rpm', 10)
        self.right_rpm_pub = self.create_publisher(Float64, '/right_wheel_rpm', 10)
        
        # State
        self.last_left_position = None
        self.last_right_position = None
        self.last_time = None
        
        self.get_logger().info('Wheel RPM Publisher node started.')
    
    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        
        try:
            # Find wheel joint indices
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            
            left_position = msg.position[left_idx]
            right_position = msg.position[right_idx]
            
            if self.last_time is None:
                self.last_left_position = left_position
                self.last_right_position = right_position
                self.last_time = current_time
                return
            
            # Calculate dt
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt <= 0 or dt > 1.0:
                return
            
            # Calculate angular velocities (rad/s)
            left_vel = (left_position - self.last_left_position) / dt
            right_vel = (right_position - self.last_right_position) / dt
            
            # Convert to RPM (revolutions per minute)
            left_rpm = (left_vel * 60.0) / (2.0 * math.pi)
            right_rpm = (right_vel * 60.0) / (2.0 * math.pi)
            
            # Publish
            left_msg = Float64()
            left_msg.data = left_rpm
            self.left_rpm_pub.publish(left_msg)
            
            right_msg = Float64()
            right_msg.data = right_rpm
            self.right_rpm_pub.publish(right_msg)
            
            # Update state
            self.last_left_position = left_position
            self.last_right_position = right_position
            self.last_time = current_time
            
        except (ValueError, IndexError) as e:
            pass  # Joints not found yet


def main(args=None):
    rclpy.init(args=args)
    node = WheelRpmPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
