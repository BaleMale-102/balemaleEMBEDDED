from setuptools import find_packages, setup

package_name = 'anpr_detector'

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
    maintainer='a102',
    maintainer_email='a102@ssafy.com',
    description='ANPR detection node for ROS2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'detector_node = anpr_detector.detector_node:main',  # 기존 (호환용)
            'anomaly_node = anpr_detector.anomaly_detector_node:main',  # 신규
            'ocr_node = anpr_detector.ocr_detector_node:main',  # 신규
        ],
    },
)
