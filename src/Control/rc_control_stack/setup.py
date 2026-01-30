from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rc_control_stack'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a102',
    maintainer_email='a102@ssafy.com',
    description='Control stack for autonomous RC car (Lane follow, Marker align, Park FSM)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_stack_node = rc_control_stack.control_stack_node:main',
            'safety_manager_node = rc_control_stack.safety_manager_node:main',
        ],
    },
)
