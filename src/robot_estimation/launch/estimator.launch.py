import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # IMU Filter Node
        Node(
            package='robot_estimation',
            executable='imu_filter',
            name='imu_filter',
            output='screen',
            parameters=[{
                'alpha_accel': 0.1,
                'alpha_gyro': 0.1,
                'bias_samples': 100
            }]
        ),
        
        # Complementary Filter Node
        Node(
            package='robot_estimation',
            executable='complementary_filter',
            name='complementary_filter',
            output='screen',
            parameters=[{
                'alpha': 0.98
            }]
        ),
        
        # Wheel RPM Publisher Node
        Node(
            package='robot_estimation',
            executable='wheel_rpm_publisher',
            name='wheel_rpm_publisher',
            output='screen',
            parameters=[{
                'wheel_radius': 0.1
            }]
        ),
        
        # Motion Controller Node
        Node(
            package='robot_estimation',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
            parameters=[{
                'wheel_radius': 0.1,
                'wheel_separation': 0.5
            }]
        ),
        
        # Motion Odometry Node
        Node(
            package='robot_estimation',
            executable='motion_odometry',
            name='motion_odometry',
            output='screen',
            parameters=[{
                'wheel_radius': 0.1,
                'wheel_separation': 0.5
            }]
        ),
    ])
