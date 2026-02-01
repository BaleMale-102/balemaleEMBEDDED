#!/usr/bin/env python3
"""
simulation.launch.py - Simulation Mode Launch File

Launches all nodes in simulation mode for testing without hardware.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')

    # Include main launch with simulation=true
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'simulation': 'true',
        }.items()
    )

    return LaunchDescription([
        robot_launch,
    ])
