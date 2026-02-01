#!/usr/bin/env python3
"""
robot.launch.py - Main Robot Launch File

Launches all nodes for autonomous operation.

Arguments:
    simulation: Run in simulation mode (default: false)
    show_debug: Show debug images (default: true)
    config_file: Path to parameter file (default: robot_params.yaml)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('robot_bringup')

    # Launch arguments
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

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'robot_params.yaml'),
        description='Path to parameter file'
    )

    # Get launch configurations
    simulation = LaunchConfiguration('simulation')
    show_debug = LaunchConfiguration('show_debug')
    config_file = LaunchConfiguration('config_file')

    # ==========================================
    # Perception Nodes
    # ==========================================
    perception_group = GroupAction([
        # Marker Detector
        Node(
            package='marker_detector',
            executable='detector_node',
            name='marker_detector',
            parameters=[config_file],
            remappings=[
                ('image_raw', '/camera/front/image_raw'),
                ('camera_info', '/camera/front/camera_info'),
            ],
        ),

        # Marker Tracker
        Node(
            package='marker_tracker',
            executable='tracker_node',
            name='marker_tracker',
            parameters=[config_file],
        ),

        # Lane Detector
        Node(
            package='lane_detector',
            executable='detector_node',
            name='lane_detector',
            parameters=[config_file],
            remappings=[
                ('image_raw', '/camera/bottom/image_raw'),
            ],
        ),
    ])

    # ==========================================
    # Control Nodes
    # ==========================================
    control_group = GroupAction([
        # Motion Controller
        Node(
            package='motion_controller',
            executable='controller_node',
            name='motion_controller',
            parameters=[config_file],
        ),

        # Wheel Controller
        Node(
            package='wheel_controller',
            executable='controller_node',
            name='wheel_controller',
            parameters=[
                config_file,
                {'simulation': simulation},
            ],
        ),
    ])

    # ==========================================
    # Planning Nodes
    # ==========================================
    planning_group = GroupAction([
        # Mission Manager
        Node(
            package='mission_manager',
            executable='manager_node',
            name='mission_manager',
            parameters=[config_file],
        ),

        # Server Bridge
        Node(
            package='server_bridge',
            executable='bridge_node',
            name='server_bridge',
            parameters=[
                config_file,
                {'simulation': simulation},
            ],
        ),
    ])

    return LaunchDescription([
        # Arguments
        simulation_arg,
        show_debug_arg,
        config_file_arg,

        # Node groups
        perception_group,
        control_group,
        planning_group,
    ])
