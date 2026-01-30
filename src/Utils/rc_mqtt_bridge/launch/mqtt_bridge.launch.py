from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rc_mqtt_bridge',
            executable='mqtt_bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            parameters=[
                {'mqtt.host': '192.168.35.20'},
                {'mqtt.port': 1883},
            ]
        )
    ])

