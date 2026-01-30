#!/usr/bin/env python3
"""
arduino_bridge.launch.py

Arduino Bridge 단독 실행
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    arduino_pkg = get_package_share_directory('arduino_bridge')
    
    # Config
    config = os.path.join(arduino_pkg, 'config', 'arduino_bridge.yaml')
    
    # Arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Arduino serial port'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate'
    )
    
    # Node
    arduino_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        parameters=[
            config,
            {
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        baud_arg,
        arduino_node,
    ])
