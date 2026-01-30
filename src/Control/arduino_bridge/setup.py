from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arduino_bridge'

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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='a102',
    maintainer_email='a102@ssafy.com',
    description='Arduino UART Bridge with Odometry',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = arduino_bridge.arduino_bridge_node:main',
        ],
    },
)
