#!/usr/bin/env python3
"""
bridge_node.py - Server Bridge Node

Bridges MQTT communication with ROS2 topics.

MQTT Topics:
  Subscribes:
    server/{car_id}/task_cmd - Task commands from server
    server/{car_id}/emergency - Emergency stop
  Publishes:
    car/{car_id}/status - Robot status
    car/{car_id}/pose - Robot position
    car/{car_id}/task_status - Task execution status

ROS2 Topics:
  Subscribes:
    /driving/state: DrivingState
    /server/task_status: MissionStatus
    /localization/pose: PoseStamped
  Publishes:
    /server/task_cmd: MissionCommand
    /control/drive_cmd_emergency: DriveCmd (emergency stop)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from .mqtt_client import MQTTClient, MQTTConfig


class ServerBridgeNode(Node):
    """ROS2 to MQTT bridge."""

    def __init__(self):
        super().__init__('server_bridge_node')

        # Parameters
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('car_id', 'car_01')
        self.declare_parameter('simulation', False)
        self.declare_parameter('status_rate', 2.0)

        # MQTT configuration
        config = MQTTConfig(
            host=self.get_parameter('mqtt_host').value,
            port=self.get_parameter('mqtt_port').value,
            username=self.get_parameter('mqtt_username').value,
            password=self.get_parameter('mqtt_password').value,
            client_id=self.get_parameter('car_id').value
        )
        simulation = self.get_parameter('simulation').value
        status_rate = self.get_parameter('status_rate').value

        # MQTT client
        self.mqtt = MQTTClient(config, simulation)
        self.mqtt.set_connect_callback(self._mqtt_connect_callback)

        # State
        self._last_driving_state = None
        self._last_pose = None
        self._last_task_status = None

        # Import custom interfaces
        try:
            from robot_interfaces.msg import MissionCommand, MissionStatus, DrivingState
            from robot_interfaces.msg import DetectionArray
            self._has_interface = True
            self._MissionCommand = MissionCommand
            self._MissionStatus = MissionStatus
            self._DrivingState = DrivingState
            self._DetectionArray = DetectionArray
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        # Publishers
        if self._has_interface:
            self.pub_task_cmd = self.create_publisher(
                self._MissionCommand, '/server/task_cmd', 10
            )

        self.pub_emergency = self.create_publisher(
            Twist, '/control/drive_cmd_emergency', 10
        )

        # Subscribers
        if self._has_interface:
            self.sub_driving_state = self.create_subscription(
                self._DrivingState, '/driving/state',
                self._driving_state_callback, 10
            )
            self.sub_task_status = self.create_subscription(
                self._MissionStatus, '/server/task_status',
                self._task_status_callback, 10
            )
            self.sub_detections = self.create_subscription(
                self._DetectionArray, '/perception/anpr/detections',
                self._detections_callback, 10
            )

        # ANPR plate (String) - 호환용
        self.sub_plate = self.create_subscription(
            String, '/perception/anpr/plate',
            self._plate_callback, 10
        )

        self.sub_pose = self.create_subscription(
            PoseStamped, '/localization/pose',
            self._pose_callback, 10
        )

        # Status publish timer
        self.status_timer = self.create_timer(1.0 / status_rate, self._status_timer_callback)

        # Connect to MQTT
        if self.mqtt.connect():
            self.get_logger().info(f'MQTT connecting to {config.host}:{config.port}')
            self._setup_mqtt_subscriptions()
        else:
            self.get_logger().warn('MQTT connection failed')

        self.get_logger().info('ServerBridgeNode started')

    def _setup_mqtt_subscriptions(self):
        """Setup MQTT topic subscriptions."""
        self.mqtt.subscribe('task_cmd', self._mqtt_task_cmd_callback)
        self.mqtt.subscribe('emergency', self._mqtt_emergency_callback)

    def _mqtt_connect_callback(self, connected: bool):
        """Handle MQTT connection status."""
        if connected:
            self.get_logger().info('MQTT connected')
        else:
            self.get_logger().warn('MQTT disconnected')

    def _mqtt_task_cmd_callback(self, topic: str, payload: dict):
        """Handle task command from server."""
        if not self._has_interface:
            return

        try:
            msg = self._MissionCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.command = payload.get('command', 'START')
            msg.waypoint_ids = payload.get('waypoint_ids', [])
            msg.final_goal_id = payload.get('final_goal_id', -1)
            msg.task_id = payload.get('task_id', '')
            msg.task_type = payload.get('task_type', '')

            self.pub_task_cmd.publish(msg)
            self.get_logger().info(f'Task command received: {msg.command}')

        except Exception as e:
            self.get_logger().error(f'Task command parse error: {e}')

    def _mqtt_emergency_callback(self, topic: str, payload: dict):
        """Handle emergency stop from server."""
        self.get_logger().warn('Emergency stop received!')

        # Publish zero velocity command
        msg = Twist()
        self.pub_emergency.publish(msg)

    def _driving_state_callback(self, msg):
        """Handle driving state update."""
        self._last_driving_state = msg

    def _task_status_callback(self, msg):
        """Handle task status update."""
        self._last_task_status = msg

        # Forward to MQTT
        if self.mqtt.is_connected:
            self.mqtt.publish('task_status', {
                'task_id': msg.task_id,
                'status': msg.status,
                'current_state': msg.current_state,
                'progress': msg.progress,
                'message': msg.message
            })

    def _pose_callback(self, msg: PoseStamped):
        """Handle pose update."""
        self._last_pose = msg

    def _detections_callback(self, msg):
        """Handle ANPR detections."""
        if not self.mqtt.is_connected:
            return

        # 검출 결과를 MQTT로 전송
        detections_data = []
        for det in msg.detections:
            det_dict = {
                'class_id': det.class_id,
                'class_name': det.class_name,
                'confidence': det.confidence,
                'bbox': [det.x1, det.y1, det.x2, det.y2],
            }
            # 번호판인 경우 OCR 결과 추가
            if det.class_id == 0 and det.text:
                det_dict['plate_text'] = det.text
                det_dict['has_sticker'] = det.has_sticker
            detections_data.append(det_dict)

        self.mqtt.publish('anpr/detections', {
            'num_plates': msg.num_plates,
            'num_obstacles': msg.num_obstacles,
            'detections': detections_data
        })

    def _plate_callback(self, msg: String):
        """Handle plate OCR result."""
        if not self.mqtt.is_connected:
            return

        if msg.data:
            self.mqtt.publish('anpr/plate', {
                'plate': msg.data
            })
            self.get_logger().info(f'Plate sent to server: {msg.data}')

    def _status_timer_callback(self):
        """Periodic status publish."""
        if not self.mqtt.is_connected:
            return

        # Publish robot status
        status_data = {}
        if self._last_driving_state:
            status_data['state'] = self._last_driving_state.state
            status_data['target_marker'] = self._last_driving_state.target_marker_id

        self.mqtt.publish_status('online', status_data)

        # Publish pose
        if self._last_pose:
            pos = self._last_pose.pose.position
            ori = self._last_pose.pose.orientation

            # Extract yaw from quaternion
            siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1 - 2 * (ori.y**2 + ori.z**2)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self.mqtt.publish_pose(pos.x, pos.y, yaw)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServerBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
