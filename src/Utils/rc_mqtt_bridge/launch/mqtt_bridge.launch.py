#!/usr/bin/env python3
"""
mqtt_bridge.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('rc_mqtt_bridge')
    config = os.path.join(pkg, 'config', 'mqtt_bridge.yaml')
    
    # Arguments
    car_id_arg = DeclareLaunchArgument(
        'car_id', default_value='car_01',
        description='Car ID for MQTT topics'
    )
    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host', default_value='localhost',
        description='MQTT broker host'
    )
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port', default_value='1883',
        description='MQTT broker port'
    )
    
    # Node
    mqtt_node = Node(
        package='rc_mqtt_bridge',
        executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        parameters=[
            config,
            {
                'car_id': LaunchConfiguration('car_id'),
                'mqtt_host': LaunchConfiguration('mqtt_host'),
                'mqtt_port': LaunchConfiguration('mqtt_port'),
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        car_id_arg,
        mqtt_host_arg,
        mqtt_port_arg,
        mqtt_node,
    ])
