from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rc_perception'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a102',
    maintainer_email='a102@ssafy.com',
    description='Unified perception package (lane_v2, marker, parking_line, slot_marker)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 핵심 노드
            'lane_node = rc_perception.lane_node_v2:main',
            'lane_node_v2 = rc_perception.lane_node_v2:main',
            'marker_pose_node = rc_perception.marker_pose_node:main',
            # 주차용 (stub - 실제 구현 필요)
            'parking_line_node = rc_perception.parking_line_node:main',
            'slot_marker_node = rc_perception.slot_marker_node:main',
        ],
    },
)
