from setuptools import find_packages, setup

package_name = 'motion_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kng',
    maintainer_email='henry3447@naver.com',
    description='Motion controller with lane/marker following modes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = motion_controller.controller_node:main',
        ],
    },
)
