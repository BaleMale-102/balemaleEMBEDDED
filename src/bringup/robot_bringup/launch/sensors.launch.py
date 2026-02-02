#!/usr/bin/env python3
"""
sensors.launch.py - 센서 노드 실행

실행 노드:
  - 3x USB 카메라 (camera_driver)
  - Arduino 드라이버 (arduino_driver)
  - Static TF (robot frames)

Arguments:
    simulation: 시뮬레이션 모드 (default: false)
    cam_front_dev: 전방 카메라 디바이스 (default: /dev/video0)
    cam_bottom_dev: 하단 카메라 디바이스 (default: /dev/video2)
    cam_side_dev: 측면 카메라 디바이스 (default: /dev/video4)
    arduino_port: Arduino 시리얼 포트 (default: /dev/ttyUSB0)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    bringup_dir = get_package_share_directory('robot_bringup')

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

    cam_bottom_dev_arg = DeclareLaunchArgument(
        'cam_bottom_dev',
        default_value='/dev/video2',
        description='Bottom camera device'
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
        parameters=[{
            'device': LaunchConfiguration('cam_front_dev'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'frame_id': 'camera_front',
            'camera_name': 'cam_front',
            'simulate': simulation,
        }],
        output='screen'
    )

    # Bottom camera (lane detection)
    cam_bottom = Node(
        package='camera_driver',
        executable='camera_node',
        name='cam_bottom',
        parameters=[{
            'device': LaunchConfiguration('cam_bottom_dev'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'frame_id': 'camera_bottom',
            'camera_name': 'cam_bottom',
            'simulate': simulation,
        }],
        output='screen'
    )

    # Side camera (parking)
    cam_side = Node(
        package='camera_driver',
        executable='camera_node',
        name='cam_side',
        parameters=[{
            'device': LaunchConfiguration('cam_side_dev'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'frame_id': 'camera_side',
            'camera_name': 'cam_side',
            'simulate': simulation,
        }],
        output='screen'
    )

    # ==========================================
    # Arduino Driver
    # ==========================================
    arduino_node = Node(
        package='arduino_driver',
        executable='arduino_node',
        name='arduino_driver',
        parameters=[{
            'port': LaunchConfiguration('arduino_port'),
            'baudrate': 115200,
            'timeout': 0.1,
            'simulate': simulation,
            'imu_frame_id': 'imu_link',
            'watchdog_timeout': 0.5,
        }],
        output='screen'
    )

    # ==========================================
    # Static TF Publishers
    # ==========================================

    # base_link -> camera_front (5cm forward, 1cm up)
    tf_base_to_cam_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_front',
        arguments=['0.05', '0', '0.01', '0', '0', '0', 'base_link', 'camera_front']
    )

    # base_link -> camera_bottom (아래 방향, pitch 90도)
    tf_base_to_cam_bottom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_bottom',
        arguments=['0', '0', '0.02', '0', '1.5708', '0', 'base_link', 'camera_bottom']
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
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link']
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
        cam_bottom_dev_arg,
        cam_side_dev_arg,
        arduino_port_arg,

        # Cameras
        cam_front,
        cam_bottom,
        cam_side,

        # Arduino
        arduino_node,

        # Static TF
        tf_base_to_cam_front,
        tf_base_to_cam_bottom,
        tf_base_to_cam_side,
        tf_base_to_imu,
        tf_odom_to_base,
    ])
