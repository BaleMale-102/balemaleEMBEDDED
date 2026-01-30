#!/usr/bin/env python3
"""
control.launch.py

제어 노드들 실행:
  - control_stack_node (자율주행 컨트롤러 + DrivingState)
  - safety_manager_node (MUX + IMU Heading Hold)
  - arduino_bridge_node (UART 통신 + Odometry)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    control_stack_pkg = get_package_share_directory('rc_control_stack')
    arduino_pkg = get_package_share_directory('arduino_bridge')
    
    # Config files
    control_config = os.path.join(control_stack_pkg, 'config', 'control_stack.yaml')
    arduino_config = os.path.join(arduino_pkg, 'config', 'arduino_bridge.yaml')
    
    # Arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Arduino serial port'
    )
    
    use_heading_hold_arg = DeclareLaunchArgument(
        'use_heading_hold',
        default_value='true',
        description='Enable IMU heading hold'
    )
    
    # Nodes
    control_stack_node = Node(
        package='rc_control_stack',
        executable='control_stack_node',
        name='control_stack_node',
        parameters=[control_config],
        output='screen'
    )
    
    safety_manager_node = Node(
        package='rc_control_stack',
        executable='safety_manager_node',
        name='safety_manager_node',
        parameters=[
            control_config,
            {'use_imu_heading_hold': LaunchConfiguration('use_heading_hold')}
        ],
        output='screen'
    )
    
    arduino_bridge_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        parameters=[
            arduino_config,
            {'port': LaunchConfiguration('serial_port')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        use_heading_hold_arg,
        control_stack_node,
        safety_manager_node,
        arduino_bridge_node,
    ])
