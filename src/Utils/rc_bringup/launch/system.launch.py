# rc_bringup/launch/system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('rc_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    sensors = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors.launch.py')))
    perception = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'perception.launch.py')))
    control = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'control.launch.py')))
    utils = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'utils.launch.py')))

    return LaunchDescription([
        DeclareLaunchArgument('marker_map_yaml', default_value='/home/a102/ws/config/marker_map.yaml'),
        sensors,
        perception,
        control,
        utils,
    ])
