from setuptools import setup
from glob import glob
import os

package_name = 'mobile_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Mobile IMU data receiver and processor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_receiver = mobile_imu.sensor_receiver:main',
            'mobile_imu_filter = mobile_imu.mobile_imu_filter:main',
            'position_estimator = mobile_imu.position_estimator:main',
        ],
    },
)
