from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rc_mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a102',
    maintainer_email='a102@todo.todo',
    description='MQTT <-> ROS2 bridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = rc_mqtt_bridge.mqtt_bridge_node:main',
        ],
    },
)
