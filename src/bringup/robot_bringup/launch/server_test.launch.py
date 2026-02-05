#!/usr/bin/env python3
"""
server_test.launch.py - 서버 통신 테스트용 런치 파일

카메라, 아두이노 등 센서 없이 서버(MQTT)와의 통신만 테스트

Nodes:
    - server_bridge: MQTT 브릿지 (balemale backend)
    - mission_manager: 미션 FSM (서버 명령 처리)

Usage:
    ros2 launch robot_bringup server_test.launch.py
    ros2 launch robot_bringup server_test.launch.py simulation:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')

    # Launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'robot_params.yaml'),
        description='Path to parameter file'
    )

    marker_map_arg = DeclareLaunchArgument(
        'marker_map',
        default_value=os.path.join(bringup_dir, 'config', 'marker_map.yaml'),
        description='Path to marker map file'
    )

    # Get launch configurations
    simulation = LaunchConfiguration('simulation')
    config_file = LaunchConfiguration('config_file')
    marker_map = LaunchConfiguration('marker_map')

    # Marker map parameter
    marker_map_param = {'marker_map_yaml': marker_map}

    # ==========================================
    # Server Bridge Node (MQTT)
    # ==========================================
    server_bridge_node = Node(
        package='server_bridge',
        executable='bridge_node',
        name='server_bridge',
        parameters=[
            config_file,
            {'simulation': simulation},
        ],
        output='screen',
    )

    # ==========================================
    # Mission Manager Node
    # ==========================================
    mission_manager_node = Node(
        package='mission_manager',
        executable='manager_node',
        name='mission_manager',
        parameters=[
            config_file,
            marker_map_param,
            {
                # 센서 없이 테스트할 때 타임아웃 비활성화
                'auto_start_waiting': False,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        simulation_arg,
        config_file_arg,
        marker_map_arg,

        # Nodes
        server_bridge_node,
        mission_manager_node,
    ])
