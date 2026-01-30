#!/usr/bin/env python3
"""
localization.launch.py

Localization 노드 실행:
  - ekf_localization_node (EKF: Marker + IMU + Odom 융합)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    localization_pkg = get_package_share_directory('rc_localization')
    
    # Config files
    ekf_config = os.path.join(localization_pkg, 'config', 'ekf_localization.yaml')
    
    # Arguments
    marker_map_arg = DeclareLaunchArgument(
        'marker_map_yaml',
        default_value='',
        description='Path to marker_map.yaml'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF (map -> base_link)'
    )
    
    # Nodes
    ekf_node = Node(
        package='rc_localization',
        executable='ekf_localization_node',
        name='ekf_localization_node',
        parameters=[
            ekf_config,
            {
                'marker_map_yaml': LaunchConfiguration('marker_map_yaml'),
                'publish_tf': LaunchConfiguration('publish_tf')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        marker_map_arg,
        publish_tf_arg,
        ekf_node,
    ])
