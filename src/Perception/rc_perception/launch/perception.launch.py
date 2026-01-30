from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Path to perception.yaml'
        ),

        Node(
            package='rc_perception',
            executable='lane_node',
            name='lane_node',
            output='screen',
            parameters=[params] if params else []
        ),

        Node(
            package='rc_perception',
            executable='marker_node',
            name='marker_node',
            output='screen',
            parameters=[params] if params else []
        ),

        Node(
            package='rc_perception',
            executable='parking_line_node',
            name='parking_line_node',
            output='screen',
            parameters=[params] if params else []
        ),

        Node(
            package='rc_perception',
            executable='slot_marker_pose_node',
            name='slot_marker_pose_node',
            output='screen',
            parameters=[params] if params else []
        ),
    ])
