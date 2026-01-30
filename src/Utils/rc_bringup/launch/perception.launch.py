# rc_bringup/launch/perception.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_perception = LaunchConfiguration('use_perception')
    use_lane = LaunchConfiguration('use_lane')
    use_marker_pose_front = LaunchConfiguration('use_marker_pose_front')
    use_parking_line = LaunchConfiguration('use_parking_line')
    use_slot_marker = LaunchConfiguration('use_slot_marker')

    # topics + debug
    lane_image_topic = LaunchConfiguration('lane_image_topic')
    lane_show_debug = LaunchConfiguration('lane_show_debug')
    bottom_camera_info_topic = LaunchConfiguration('bottom_camera_info_topic')

    marker_pose_front_image_topic = LaunchConfiguration('marker_pose_front_image_topic')
    marker_pose_front_camera_info_topic = LaunchConfiguration('marker_pose_front_camera_info_topic')
    marker_pose_front_show_debug = LaunchConfiguration('marker_pose_front_show_debug')

    parking_line_image_topic = LaunchConfiguration('parking_line_image_topic')
    parking_line_show_debug = LaunchConfiguration('parking_line_show_debug')

    slot_image_topic = LaunchConfiguration('slot_image_topic')
    slot_marker_show_debug = LaunchConfiguration('slot_marker_show_debug')

    side_camera_info_topic = LaunchConfiguration('side_camera_info_topic')

    # frames
    base_frame = LaunchConfiguration('tf_base_frame')
    front_frame = LaunchConfiguration('tf_front_frame')
    side_frame = LaunchConfiguration('tf_side_frame')

    default_perception_yaml = os.path.join(get_package_share_directory('rc_perception'), 'config', 'perception.yaml')
    perception_yaml = LaunchConfiguration('perception_yaml')

    lane_node = Node(
        package='rc_perception', executable='lane_node', name='lane_node', output='screen',
        parameters=[perception_yaml, {'image_topic': lane_image_topic, 'camera_info_topic': bottom_camera_info_topic, 'show_debug': lane_show_debug}],
        condition=IfCondition(PythonExpression(["'", use_perception, "'=='true' and '", use_lane, "'=='true'"])),
    )

    marker_pose_node_front = Node(
        package='rc_perception', executable='marker_pose_node', name='marker_pose_node_front', output='screen',
        parameters=[perception_yaml, {
            'image_topic': marker_pose_front_image_topic,
            'camera_info_topic': marker_pose_front_camera_info_topic,
            'show_debug': marker_pose_front_show_debug,
            'camera_frame': front_frame,
            'base_frame': base_frame,
            'marker_length_m': 0.02,
            'pose_topic': '/perception/marker_pose',
            'publish_tf': True,
            'tf_parent_frame': base_frame,
            'tf_child_frame': 'front_marker',
        }],
        condition=IfCondition(PythonExpression(["'", use_perception, "'=='true' and '", use_marker_pose_front, "'=='true'"])),
    )

    parking_line_node = Node(
        package='rc_perception', executable='parking_line_node', name='parking_line_node', output='screen',
        parameters=[perception_yaml, {'image_topic': parking_line_image_topic, 'camera_info_topic': side_camera_info_topic, 'show_debug': parking_line_show_debug}],
        condition=IfCondition(PythonExpression(["'", use_perception, "'=='true' and '", use_parking_line, "'=='true'"])),
    )

    slot_marker_node = Node(
        package='rc_perception', executable='slot_marker_node', name='slot_marker_node', output='screen',
        parameters=[perception_yaml, {
            'image_topic': slot_image_topic,
            'camera_info_topic': side_camera_info_topic,
            'pose_topic': '/perception/slot_marker_pose',
            'base_frame': base_frame,
            'camera_frame': side_frame,
            'show_debug': slot_marker_show_debug,
        }],
        condition=IfCondition(PythonExpression(["'", use_perception, "'=='true' and '", use_slot_marker, "'=='true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_perception', default_value='true'),
        DeclareLaunchArgument('use_lane', default_value='true'),
        DeclareLaunchArgument('use_marker_pose_front', default_value='true'),
        DeclareLaunchArgument('use_parking_line', default_value='true'),
        DeclareLaunchArgument('use_slot_marker', default_value='true'),

        DeclareLaunchArgument('perception_yaml', default_value=default_perception_yaml),

        # camera_info topics
        DeclareLaunchArgument('bottom_camera_info_topic', default_value='/cam_bottom/camera_info'),
        DeclareLaunchArgument('side_camera_info_topic', default_value='/cam_side/camera_info'),
        DeclareLaunchArgument('marker_pose_front_camera_info_topic', default_value='/cam_front/camera_info'),

        # image topics
        DeclareLaunchArgument('lane_image_topic', default_value='/cam_bottom/image_raw'),
        DeclareLaunchArgument('parking_line_image_topic', default_value='/cam_side/image_raw'),
        DeclareLaunchArgument('slot_image_topic', default_value='/cam_side/image_raw'),
        DeclareLaunchArgument('marker_pose_front_image_topic', default_value='/cam_front/image_raw'),

        # debug
        DeclareLaunchArgument('lane_show_debug', default_value='false'),
        DeclareLaunchArgument('parking_line_show_debug', default_value='false'),
        DeclareLaunchArgument('slot_marker_show_debug', default_value='false'),
        DeclareLaunchArgument('marker_pose_front_show_debug', default_value='true'),

        # frames (system에서 같이 쓰기 위해)
        DeclareLaunchArgument('tf_base_frame', default_value='base_link'),
        DeclareLaunchArgument('tf_front_frame', default_value='camera_front'),
        DeclareLaunchArgument('tf_side_frame', default_value='camera_side'),

        lane_node,
        marker_pose_node_front,
        parking_line_node,
        slot_marker_node,
    ])
