from setuptools import setup

package_name = 'rc_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/config', ['config/perception.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a102',
    maintainer_email='a102@todo.todo',
    description='Unified perception package (lane/marker/parking_line/slot_marker_pose)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_node = rc_perception.lane_node:main',
            'parking_line_node = rc_perception.parking_line_node:main',
            'slot_marker_node = rc_perception.slot_marker_node:main',
            'marker_pose_node = rc_perception.marker_pose_node:main',
        ],
    },
)
