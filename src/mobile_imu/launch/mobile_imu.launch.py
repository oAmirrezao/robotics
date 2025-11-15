from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Static TF: map -> mobile_imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_mobile_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'mobile_imu_link']
        ),
        
        # Sensor Receiver Node
        Node(
            package='mobile_imu',
            executable='sensor_receiver',
            name='sensor_receiver',
            output='screen'
        ),
        
        # Mobile IMU Filter Node
        Node(
            package='mobile_imu',
            executable='mobile_imu_filter',
            name='mobile_imu_filter',
            output='screen',
            parameters=[{
                'alpha_accel': 0.2,
                'alpha_gyro': 0.2,
                'bias_samples': 200
            }]
        ),
        
        # Position Estimator Node
        Node(
            package='mobile_imu',
            executable='position_estimator',
            name='position_estimator',
            output='screen',
            parameters=[{
                'alpha': 0.98
            }]
        ),
    ])
