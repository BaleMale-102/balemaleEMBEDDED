from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rc_imu_mpu6050'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='a102',
    maintainer_email='a102@ssafy.com',
    description='MPU6050 I2C IMU driver with complementary filter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_mpu6050_node = rc_imu_mpu6050.imu_mpu6050_node:main',
        ],
    },
)
