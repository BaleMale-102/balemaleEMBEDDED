#!/usr/bin/env python3
"""
perception.launch.py

인식 노드들 실행:
  - lane_node_v2 (개선된 차선 인식)
  - marker_pose_node (ArUco 마커)
  - parking_line_node (주차선)
  - slot_marker_node (주차칸 마커)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    perception_pkg = get_package_share_directory('rc_perception')
    
    # Config files
    lane_config = os.path.join(perception_pkg, 'config', 'lane_v2.yaml')
    
    # Arguments
    use_lane_v2_arg = DeclareLaunchArgument(
        'use_lane_v2',
        default_value='true',
        description='Use improved lane_node_v2'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='Show debug windows'
    )
    
    # Nodes
    lane_node = Node(
        package='rc_perception',
        executable='lane_node_v2',
        name='lane_node',
        parameters=[
            lane_config,
            {'show_debug': LaunchConfiguration('show_debug')}
        ],
        output='screen'
    )
    
    marker_pose_node = Node(
        package='rc_perception',
        executable='marker_pose_node',
        name='marker_pose_node',
        output='screen'
    )
    
    parking_line_node = Node(
        package='rc_perception',
        executable='parking_line_node',
        name='parking_line_node',
        output='screen'
    )
    
    slot_marker_node = Node(
        package='rc_perception',
        executable='slot_marker_node',
        name='slot_marker_node',
        output='screen'
    )
    
    return LaunchDescription([
        use_lane_v2_arg,
        show_debug_arg,
        lane_node,
        marker_pose_node,
        parking_line_node,
        slot_marker_node,
    ])
