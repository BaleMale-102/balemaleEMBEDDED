#!/usr/bin/env python3
"""
sensors.launch.py - 센서 노드 실행

실행 노드:
  - USB 카메라 (camera_driver): cam_front, cam_side
  - Arduino 드라이버 (arduino_driver)
  - IMU 드라이버 (imu_driver)
  - Static TF (robot frames)

Arguments:
    simulation: 시뮬레이션 모드 (default: false)
    cam_front_dev: 전방 카메라 디바이스 (default: /dev/video0)
    cam_side_dev: 측면 카메라 디바이스 (default: /dev/video4)
    arduino_port: Arduino 시리얼 포트 (default: /dev/ttyUSB0)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    bringup_dir = get_package_share_directory('robot_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'robot_params.yaml')
    cam_front_calib = os.path.join(bringup_dir, 'config', 'cam_front_calib.yaml')

    # ==========================================
    # Launch Arguments
    # ==========================================
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode'
    )

    cam_front_dev_arg = DeclareLaunchArgument(
        'cam_front_dev',
        default_value='/dev/video0',
        description='Front camera device'
    )

    cam_side_dev_arg = DeclareLaunchArgument(
        'cam_side_dev',
        default_value='/dev/video4',
        description='Side camera device'
    )

    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyUSB0',
        description='Arduino serial port'
    )

    # Get configurations
    simulation = LaunchConfiguration('simulation')

    # ==========================================
    # Camera Nodes
    # ==========================================

    # Front camera (marker detection)
    cam_front = Node(
        package='camera_driver',
        executable='camera_node',
        name='cam_front',
        parameters=[
            config_file,
            {
                'device': LaunchConfiguration('cam_front_dev'),
                'simulate': simulation,
                'calibration_file': cam_front_calib,
            }
        ],
        output='screen'
    )

    # Side camera (parking)
    cam_side = Node(
        package='camera_driver',
        executable='camera_node',
        name='cam_side',
        parameters=[
            config_file,
            {
                'device': LaunchConfiguration('cam_side_dev'),
                'simulate': simulation,
            }
        ],
        output='screen'
    )

    # ==========================================
    # Arduino Driver
    # ==========================================
    arduino_node = Node(
        package='arduino_driver',
        executable='arduino_node',
        name='arduino_driver',
        parameters=[
            config_file,
            {
                'port': LaunchConfiguration('arduino_port'),
                'simulate': simulation,
            }
        ],
        output='screen'
    )

    # ==========================================
    # IMU Driver (MPU6050 via I2C)
    # ==========================================
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        parameters=[config_file],
        output='screen'
    )

    # ==========================================
    # Static TF Publishers
    # ==========================================

    # base_link -> camera_front (10cm forward, 1cm up)
    tf_base_to_cam_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_front',
        arguments=['0.10', '0', '0.01', '0', '0', '0', 'base_link', 'camera_front']
    )

    # base_link -> camera_side (오른쪽, yaw -90도)
    tf_base_to_cam_side = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_side',
        arguments=['0', '-0.05', '0.03', '0', '0', '-1.5708', 'base_link', 'camera_side']
    )

    # base_link -> imu_link
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # odom -> base_link (identity, will be updated by localization)
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        # Arguments
        simulation_arg,
        cam_front_dev_arg,
        cam_side_dev_arg,
        arduino_port_arg,

        # Cameras
        cam_front,
        cam_side,

        # Arduino
        arduino_node,

        # IMU
        imu_node,

        # Static TF
        tf_base_to_cam_front,
        tf_base_to_cam_side,
        tf_base_to_imu,
        tf_odom_to_base,
    ])
