#!/usr/bin/env python3
"""
sensors.launch.py

센서 노드들 실행:
  - 3x USB 카메라 (v4l2_camera)
  - IMU (mpu6050)
  - Static TF (robot frames)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    bringup_pkg = get_package_share_directory('rc_bringup')
    
    # Arguments
    cam_front_dev_arg = DeclareLaunchArgument(
        'cam_front_dev', default_value='/dev/video0',
        description='Front camera device'
    )
    cam_bottom_dev_arg = DeclareLaunchArgument(
        'cam_bottom_dev', default_value='/dev/video2',
        description='Bottom camera device'
    )
    cam_side_dev_arg = DeclareLaunchArgument(
        'cam_side_dev', default_value='/dev/video4',
        description='Side camera device'
    )
    
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus', default_value='7',
        description='I2C bus for IMU'
    )
    
    # ===== Cameras (v4l2_camera) =====
    cam_front = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam_front',
        namespace='cam_front',
        parameters=[{
            'video_device': LaunchConfiguration('cam_front_dev'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_front',
        }],
        remappings=[
            ('image_raw', '/cam_front/image_raw'),
            ('camera_info', '/cam_front/camera_info'),
        ],
        output='screen'
    )
    
    cam_bottom = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam_bottom',
        namespace='cam_bottom',
        parameters=[{
            'video_device': LaunchConfiguration('cam_bottom_dev'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_bottom',
        }],
        remappings=[
            ('image_raw', '/cam_bottom/image_raw'),
            ('camera_info', '/cam_bottom/camera_info'),
        ],
        output='screen'
    )
    
    cam_side = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam_side',
        namespace='cam_side',
        parameters=[{
            'video_device': LaunchConfiguration('cam_side_dev'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_side',
        }],
        remappings=[
            ('image_raw', '/cam_side/image_raw'),
            ('camera_info', '/cam_side/camera_info'),
        ],
        output='screen'
    )
    
    # ===== IMU =====
    imu_node = Node(
        package='rc_imu_mpu6050',
        executable='imu_mpu6050_node',
        name='imu_node',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': 0x68,
            'publish_rate_hz': 100.0,
            'frame_id': 'imu_link',
        }],
        output='screen'
    )
    
    # ===== Static TF =====
    # base_link -> camera_front (6cm forward)
    tf_base_to_cam_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_front',
        arguments=['0.05', '0', '0.01', '0', '0', '0', 'base_link', 'camera_front']
    )
    
    # base_link -> camera_bottom (아래 방향)
    tf_base_to_cam_bottom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_bottom',
        arguments=['0', '0', '0.02', '0', '1.57', '0', 'base_link', 'camera_bottom']
    )
    
    # base_link -> camera_side (오른쪽)
    tf_base_to_cam_side = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam_side',
        arguments=['0', '-0.05', '0.03', '0', '0', '-1.57', 'base_link', 'camera_side']
    )
    
    # base_link -> imu_link
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    return LaunchDescription([
        # Arguments
        cam_front_dev_arg,
        cam_bottom_dev_arg,
        cam_side_dev_arg,
        i2c_bus_arg,
        
        # Cameras
        cam_front,
        cam_bottom,
        cam_side,
        
        # IMU
        imu_node,
        
        # Static TF
        tf_base_to_cam_front,
        tf_base_to_cam_bottom,
        tf_base_to_cam_side,
        tf_base_to_imu,
    ])
