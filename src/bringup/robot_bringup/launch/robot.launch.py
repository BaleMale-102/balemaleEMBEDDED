#!/usr/bin/env python3
"""
robot.launch.py - Main Robot Launch File

Launches all nodes for autonomous operation (without sensors).
Use system.launch.py to launch everything including sensors.

Arguments:
    simulation: Run in simulation mode (default: false)
    show_debug: Show debug images (default: true)
    config_file: Path to parameter file (default: robot_params.yaml)

Nodes:
    Perception: marker_detector (front), side_marker_detector, marker_tracker, slot_line_detector
    Control: motion_controller, loader_driver
    Planning: mission_manager, server_bridge
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

    marker_map_arg = DeclareLaunchArgument(
        'marker_map',
        default_value=os.path.join(bringup_dir, 'config', 'marker_map.yaml'),
        description='Path to marker map file'
    )

    # Get launch configurations
    simulation = LaunchConfiguration('simulation')
    show_debug = LaunchConfiguration('show_debug')
    config_file = LaunchConfiguration('config_file')
    marker_map = LaunchConfiguration('marker_map')

    # Marker map parameter dict for nodes that need it
    marker_map_param = {'marker_map_yaml': marker_map}

    # X11 환경변수 (SSH X11 forwarding용)
    display_env = os.environ.get('DISPLAY', ':0')
    x11_env = {
        'DISPLAY': display_env,
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'QT_X11_NO_MITSHM': '1',
    }

    # ==========================================
    # Perception Nodes
    # ==========================================
    perception_group = GroupAction([
        # Marker Detector (전방 카메라)
        Node(
            package='marker_detector',
            executable='detector_node',
            name='marker_detector',
            parameters=[config_file, {'show_debug_window': show_debug}],
            remappings=[
                ('image_raw', '/cam_front/image_raw'),
                ('camera_info', '/cam_front/camera_info'),
            ],
            additional_env=x11_env,
        ),

        # Side Marker Detector (측면 카메라 - 주차용)
        Node(
            package='marker_detector',
            executable='detector_node',
            name='side_marker_detector',
            parameters=[
                config_file,
                {
                    'show_debug_window': False,  # 측면 카메라는 디버그 창 끄기
                    'image_topic': '/cam_side/image_raw',
                    'camera_info_topic': '/cam_side/camera_info',
                    'output_topic': '/perception/side_markers',
                    'frame_id': 'camera_side_link',
                }
            ],
            remappings=[
                ('image_raw', '/cam_side/image_raw'),
                ('camera_info', '/cam_side/camera_info'),
                ('/perception/markers', '/perception/side_markers'),
            ],
            additional_env=x11_env,
        ),

        # Marker Tracker
        Node(
            package='marker_tracker',
            executable='tracker_node',
            name='marker_tracker',
            parameters=[config_file],
        ),

        # Slot Line Detector (노란 직사각형 검출 - 주차용)
        Node(
            package='slot_line_detector',
            executable='detector_node',
            name='slot_line_detector',
            parameters=[
                config_file,
                {
                    'publish_debug_image': True,
                    'show_debug_window': False,
                }
            ],
            additional_env=x11_env,
        ),

        # Lane Detector (비활성화 - 마커 전용 주행)
        # Node(
        #     package='lane_detector',
        #     executable='detector_node',
        #     name='lane_detector',
        #     parameters=[config_file, {'show_debug_window': show_debug}],
        #     remappings=[
        #         ('image_raw', '/cam_front/image_raw'),
        #     ],
        #     additional_env=x11_env,
        # ),

        # ANPR + 장애물 검출 (별도 터미널에서 실행 - conda anpr_310 환경)
        # ros2 run anpr_detector detector_node --ros-args -p show_debug_window:=true
    ])

    # ==========================================
    # Control Nodes
    # ==========================================
    # Note: arduino_driver is launched in sensors.launch.py
    control_group = GroupAction([
        # Motion Controller
        Node(
            package='motion_controller',
            executable='controller_node',
            name='motion_controller',
            parameters=[config_file],
        ),

        # Loader Driver (차량 적재 메커니즘)
        Node(
            package='loader_driver',
            executable='loader_node',
            name='loader_driver',
            parameters=[
                config_file,
                {'simulate': simulation},
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
            parameters=[config_file, marker_map_param],
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
        marker_map_arg,

        # Node groups
        perception_group,
        control_group,
        planning_group,
    ])
