# rc_bringup/launch/control.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_localization = LaunchConfiguration('use_localization')
    use_mission = LaunchConfiguration('use_mission')
    use_control_stack = LaunchConfiguration('use_control_stack')
    use_safety_manager = LaunchConfiguration('use_safety_manager')
    use_arduino_bridge = LaunchConfiguration('use_arduino_bridge')

    default_control_stack_yaml = os.path.join(get_package_share_directory('rc_control_stack'), 'config', 'control_stack.yaml')
    default_safety_manager_yaml = os.path.join(get_package_share_directory('rc_control_stack'), 'config', 'safety_manager.yaml')
    default_mission_yaml = os.path.join(get_package_share_directory('rc_mission'), 'config', 'mission_manager.yaml')
    default_localization_yaml = os.path.join(get_package_share_directory('rc_localization'), 'config', 'localization.yaml')
    default_arduino_bridge_yaml = os.path.join(get_package_share_directory('arduino_bridge'), 'config', 'arduino_bridge.yaml')

    control_stack_yaml = LaunchConfiguration('control_stack_yaml')
    safety_manager_yaml = LaunchConfiguration('safety_manager_yaml')
    mission_yaml = LaunchConfiguration('mission_yaml')
    localization_yaml = LaunchConfiguration('localization_yaml')
    arduino_bridge_yaml = LaunchConfiguration('arduino_bridge_yaml')

    marker_map_yaml = LaunchConfiguration('marker_map_yaml')
    arduino_port = LaunchConfiguration('arduino_port')
    arduino_baud = LaunchConfiguration('arduino_baud')

    localization_node = Node(
        package='rc_localization', executable='localization_node', name='localization_node', output='screen',
        parameters=[localization_yaml, {'marker_map_yaml': marker_map_yaml}],
        condition=IfCondition(use_localization),
    )

    mission_node = Node(
        package='rc_mission', executable='mission_manager_node', name='mission_manager_node', output='screen',
        parameters=[mission_yaml, {'marker_map_yaml': marker_map_yaml}],
        condition=IfCondition(use_mission),
    )

    control_stack_node = Node(
        package='rc_control_stack', executable='control_stack_node', name='control_stack_node', output='screen',
        parameters=[control_stack_yaml],
        condition=IfCondition(use_control_stack),
    )

    safety_manager_node = Node(
        package='rc_control_stack', executable='safety_manager_node', name='safety_manager_node', output='screen',
        parameters=[safety_manager_yaml],
        condition=IfCondition(use_safety_manager),
    )

    arduino_bridge_node = Node(
        package='arduino_bridge', executable='arduino_bridge_node', name='arduino_bridge_node', output='screen',
        parameters=[arduino_bridge_yaml, {'port': arduino_port, 'baud': arduino_baud}],
        condition=IfCondition(use_arduino_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument('use_mission', default_value='true'),
        DeclareLaunchArgument('use_control_stack', default_value='true'),
        DeclareLaunchArgument('use_safety_manager', default_value='true'),
        DeclareLaunchArgument('use_arduino_bridge', default_value='true'),

        DeclareLaunchArgument('control_stack_yaml', default_value=default_control_stack_yaml),
        DeclareLaunchArgument('safety_manager_yaml', default_value=default_safety_manager_yaml),
        DeclareLaunchArgument('mission_yaml', default_value=default_mission_yaml),
        DeclareLaunchArgument('localization_yaml', default_value=default_localization_yaml),

        DeclareLaunchArgument('marker_map_yaml', default_value='/home/a102/ws/config/marker_map.yaml'),

        DeclareLaunchArgument('arduino_bridge_yaml', default_value=default_arduino_bridge_yaml),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arduino_baud', default_value='115200'),

        localization_node,
        mission_node,
        control_stack_node,
        safety_manager_node,
        arduino_bridge_node,
    ])
