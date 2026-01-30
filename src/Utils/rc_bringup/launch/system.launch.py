#!/usr/bin/env python3
"""
system.launch.py

전체 시스템 실행:
  1. Sensors (카메라, IMU)
  2. Perception (lane_v2, marker, parking_line, slot_marker)
  3. Localization (EKF)
  4. Control (control_stack, safety_manager, arduino_bridge)
  5. Mission (mission_manager)

사용법:
  ros2 launch rc_bringup system.launch.py
  ros2 launch rc_bringup system.launch.py marker_map_yaml:=/path/to/marker_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== Package paths =====
    bringup_pkg = get_package_share_directory('rc_bringup')
    perception_pkg = get_package_share_directory('rc_perception')
    control_stack_pkg = get_package_share_directory('rc_control_stack')
    localization_pkg = get_package_share_directory('rc_localization')
    arduino_pkg = get_package_share_directory('arduino_bridge')
    
    # ===== Arguments =====
    marker_map_arg = DeclareLaunchArgument(
        'marker_map_yaml',
        default_value=os.path.join(bringup_pkg, 'config', 'marker_map.yaml'),
        description='Path to marker_map.yaml'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Arduino serial port'
    )
    
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Use EKF localization'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='Show perception debug windows'
    )
    
    # ===== Include launches =====
    
    # Sensors (카메라, IMU, TF)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sensors.launch.py')
        )
    )
    
    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'show_debug': LaunchConfiguration('show_debug')
        }.items()
    )
    
    # Localization (EKF)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'marker_map_yaml': LaunchConfiguration('marker_map_yaml')
        }.items()
    )
    
    # Control
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_stack_pkg, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )
    
    # Mission Manager (단독 노드)
    mission_node = Node(
        package='rc_mission',
        executable='mission_manager_node',
        name='mission_manager_node',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'mission.yaml'),
            {'marker_map_yaml': LaunchConfiguration('marker_map_yaml')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        marker_map_arg,
        serial_port_arg,
        use_ekf_arg,
        show_debug_arg,
        
        # Launches
        sensors_launch,
        perception_launch,
        localization_launch,
        control_launch,
        mission_node,
    ])
