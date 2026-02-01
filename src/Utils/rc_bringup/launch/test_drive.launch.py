#!/usr/bin/env python3
"""
test_drive.launch.py

마커 기반 주행 테스트용 launch

사용법:
  # 기본 (debug창 없음)
  ros2 launch rc_bringup test_drive.launch.py

  # debug창 켜기
  ros2 launch rc_bringup test_drive.launch.py show_debug:=true

  # 라인 인식 끄기
  ros2 launch rc_bringup test_drive.launch.py use_lane:=false

실행 후:
  # route_follower 실행
  python3 route_follower.py --ros-args -p route:="[1]"
  
  # 시작
  ros2 topic pub /route/enable std_msgs/Bool "data: true" --once
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    # ===== Arguments =====
    show_debug_arg = DeclareLaunchArgument(
        'show_debug', default_value='true',
        description='Show debug windows'
    )
    
    use_lane_arg = DeclareLaunchArgument(
        'use_lane', default_value='true',
        description='Enable lane detection (for turning)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Arduino serial port'
    )
    
    front_cam_arg = DeclareLaunchArgument(
        'front_cam', default_value='/dev/video0',
        description='Front camera device'
    )
    
    bottom_cam_arg = DeclareLaunchArgument(
        'bottom_cam', default_value='/dev/video2',
        description='Bottom camera device'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.04',
        description='ArUco marker size (m)'
    )
    
    turn_invert_arg = DeclareLaunchArgument(
        'turn_invert', default_value='false',
        description='Invert turn direction'
    )
    
    extra_forward_arg = DeclareLaunchArgument(
        'extra_forward', default_value='1.2',
        description='Extra forward time after marker lost (sec)'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='false',
        description='Enable IMU'
    )
    
    # ===== Nodes =====
    
    # Front Camera (마커 인식용)
    front_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam_front',
        namespace='cam_front',
        parameters=[{
            'video_device': LaunchConfiguration('front_cam'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_front',
        }],
        output='screen'
    )
    
    # Bottom Camera (라인 인식용)
    bottom_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam_bottom',
        namespace='cam_bottom',
        parameters=[{
            'video_device': LaunchConfiguration('bottom_cam'),
            'image_size': [640, 480],
            'camera_frame_id': 'camera_bottom',
        }],
        condition=IfCondition(LaunchConfiguration('use_lane')),
        output='screen'
    )
    
    # Marker Pose Node
    marker_pose_node = Node(
        package='rc_perception',
        executable='marker_pose_node',
        name='marker_pose_node',
        parameters=[{
            'image_topic': '/cam_front/image_raw',
            'marker_size': LaunchConfiguration('marker_size'),
            'show_debug': LaunchConfiguration('show_debug'),
        }],
        output='screen'
    )
    
    # Lane Node (턴 종료 판단용)
    lane_node = Node(
        package='rc_perception',
        executable='lane_node_v2',
        name='lane_node_v2',
        parameters=[{
            'image_topic': '/cam_bottom/image_raw',
            'show_debug': LaunchConfiguration('show_debug'),
        }],
        condition=IfCondition(LaunchConfiguration('use_lane')),
        output='screen'
    )
    
    # IMU Node
    imu_node = Node(
        package='rc_imu_mpu6050',
        executable='imu_node',
        name='imu_node',
        parameters=[{
            'i2c_bus': 1,
            'publish_rate': 50.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_imu')),
        output='screen'
    )
    
    # Arduino Bridge
    arduino_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        parameters=[{
            'port': LaunchConfiguration('serial_port'),
            'baud': 115200,
        }],
        output='screen'
    )
    
    # Safety Manager
    safety_node = Node(
        package='rc_control_stack',
        executable='safety_manager_node',
        name='safety_manager_node',
        output='screen'
    )
    
    # Control Stack
    control_node = Node(
        package='rc_control_stack',
        executable='control_stack_node',
        name='control_stack_node',
        parameters=[{
            # 테스트 확정 파라미터
            'drive_vx': 0.003,
            'drive_vy_gain': -0.03,
            'drive_max_vy': 0.005,
            'reach_distance': 0.30,
            'extra_forward_sec': LaunchConfiguration('extra_forward'),
            'turn_wz': 0.08,  # 느리게
            'turn_wz_invert': LaunchConfiguration('turn_invert'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        show_debug_arg,
        use_lane_arg,
        serial_port_arg,
        front_cam_arg,
        bottom_cam_arg,
        marker_size_arg,
        turn_invert_arg,
        extra_forward_arg,
        use_imu_arg,
        
        # Nodes
        front_camera_node,
        bottom_camera_node,
        marker_pose_node,
        lane_node,
        imu_node,
        arduino_node,
        safety_node,
        control_node,
    ])
