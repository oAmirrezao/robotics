from setuptools import setup
from glob import glob
import os

package_name = 'robot_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        f'{package_name}.filters',
        f'{package_name}.motion_model'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot estimation package with filters and motion model',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_filter = robot_estimation.filters.imu_filter:main',
            'complementary_filter = robot_estimation.filters.complementary_filter:main',
            'wheel_rpm_publisher = robot_estimation.motion_model.wheel_rpm_publisher:main',
            'motion_controller = robot_estimation.motion_model.motion_controller:main',
            'motion_odometry = robot_estimation.motion_model.motion_odometry:main',
        ],
    },
)
