# rc_bringup/launch/utils.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_human = LaunchConfiguration('use_human')
    use_mqtt_bridge = LaunchConfiguration('use_mqtt_bridge')
    mqtt_params_yaml = LaunchConfiguration('mqtt_params_yaml')

    human_node = Node(
        package='rc_human_interface', executable='keyboard_teleop_node',
        name='keyboard_teleop_node', output='screen',
        condition=IfCondition(use_human),
    )

    mqtt_bridge_node = Node(
        package='rc_mqtt_bridge', executable='mqtt_bridge_node',
        name='mqtt_bridge_node', output='screen',
        parameters=[mqtt_params_yaml],
        condition=IfCondition(use_mqtt_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_human', default_value='false'),
        DeclareLaunchArgument('use_mqtt_bridge', default_value='true'),
        DeclareLaunchArgument('mqtt_params_yaml', default_value=''),
        mqtt_bridge_node,
        human_node,
    ])
