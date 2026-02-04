#!/usr/bin/env python3
"""
system.launch.py - 전체 시스템 실행

sensors.launch.py + robot.launch.py를 함께 실행

Arguments:
    simulation: 시뮬레이션 모드 (default: false)
    show_debug: 디버그 이미지 표시 (default: true)
    cam_front_dev: 전방 카메라 디바이스 (default: /dev/video2, C920)
    cam_side_dev: 측면 카메라 디바이스 (default: /dev/video0, Brio 100)
    arduino_port: Arduino 시리얼 포트 (default: /dev/ttyUSB0)

Usage:
    ros2 launch robot_bringup system.launch.py
    ros2 launch robot_bringup system.launch.py simulation:=true
    ros2 launch robot_bringup system.launch.py cam_front_dev:=/dev/video1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ==========================================
    # Launch Arguments
    # ==========================================
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode'
    )

    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='true',
        description='Show debug images'
    )

    cam_front_dev_arg = DeclareLaunchArgument(
        'cam_front_dev',
        default_value='/dev/video0',
        description='Front camera device (C920)'
    )

    cam_side_dev_arg = DeclareLaunchArgument(
        'cam_side_dev',
        default_value='/dev/video2',
        description='Side camera device (Brio 100)'
    )

    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyUSB0',
        description='Arduino serial port'
    )

    loader_simulate_arg = DeclareLaunchArgument(
        'loader_simulate',
        default_value='true',
        description='Simulate loader (no hardware)'
    )

    # ==========================================
    # Include Sensors Launch
    # ==========================================
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'sensors.launch.py')
        ),
        launch_arguments={
            'simulation': LaunchConfiguration('simulation'),
            'cam_front_dev': LaunchConfiguration('cam_front_dev'),
            'cam_side_dev': LaunchConfiguration('cam_side_dev'),
            'arduino_port': LaunchConfiguration('arduino_port'),
        }.items()
    )

    # ==========================================
    # Include Robot Launch
    # ==========================================
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot.launch.py')
        ),
        launch_arguments={
            'simulation': LaunchConfiguration('simulation'),
            'show_debug': LaunchConfiguration('show_debug'),
            'loader_simulate': LaunchConfiguration('loader_simulate'),
        }.items()
    )

    return LaunchDescription([
        # Arguments
        simulation_arg,
        show_debug_arg,
        cam_front_dev_arg,
        cam_side_dev_arg,
        arduino_port_arg,
        loader_simulate_arg,

        # Launches
        sensors_launch,
        robot_launch,
    ])
