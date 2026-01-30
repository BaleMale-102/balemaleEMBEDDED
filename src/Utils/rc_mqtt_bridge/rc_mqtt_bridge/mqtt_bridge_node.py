#!/usr/bin/env python3
"""
mqtt_bridge_node.py

MQTT <-> ROS2 브릿지

기능:
  - 서버에서 태스크 수신 (MQTT → ROS2)
  - 태스크 상태 전송 (ROS2 → MQTT)
  - 위치/상태 주기적 전송
  - 긴급 정지 명령 수신

MQTT 토픽:
  Subscribe:
    - rc/{car_id}/task/cmd        : 태스크 명령
    - rc/{car_id}/control/emergency : 긴급 정지
    - rc/broadcast/announce       : 브로드캐스트
  
  Publish:
    - rc/{car_id}/task/status     : 태스크 상태
    - rc/{car_id}/location        : 위치 정보
    - rc/{car_id}/status          : 차량 상태

ROS2 토픽:
  Subscribe:
    - /server/task_status (TaskStatus)
    - /localization/pose (PoseStamped)
    - /driving/state (DrivingState)
  
  Publish:
    - /server/task_cmd (TaskCmd)
    - /control/drive_cmd_emergency (DriveCmd)
"""

import json
import time
import threading
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from rc_interfaces.msg import TaskCmd, TaskStatus, DriveCmd, DrivingState

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


@dataclass
class CarLocation:
    car_id: str
    x: float
    y: float
    yaw: float
    timestamp: float


@dataclass
class CarStatus:
    car_id: str
    state: str
    battery: float
    current_task_id: str
    current_marker_id: int
    timestamp: float


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')
        
        if not MQTT_AVAILABLE:
            self.get_logger().error("paho-mqtt not installed! pip install paho-mqtt")
            return
        
        # ===== Parameters =====
        self.declare_parameter('car_id', 'car_01')
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('mqtt_keepalive', 60)
        
        # ROS2 토픽
        self.declare_parameter('task_cmd_topic', '/server/task_cmd')
        self.declare_parameter('task_status_topic', '/server/task_status')
        self.declare_parameter('emergency_cmd_topic', '/control/drive_cmd_emergency')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('driving_state_topic', '/driving/state')
        
        # 전송 주기
        self.declare_parameter('location_publish_rate_hz', 5.0)
        self.declare_parameter('status_publish_rate_hz', 1.0)
        
        # Load params
        self.car_id = self.get_parameter('car_id').value
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        
        # State
        self._lock = threading.Lock()
        self._connected = False
        self._current_pose: Optional[PoseStamped] = None
        self._current_state: Optional[DrivingState] = None
        self._current_task_id = ""
        self._battery = 100.0  # TODO: 실제 배터리 연동
        
        # MQTT Client
        self.mqtt_client = mqtt.Client(client_id=f"ros2_{self.car_id}")
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        # MQTT Auth
        username = self.get_parameter('mqtt_username').value
        password = self.get_parameter('mqtt_password').value
        if username:
            self.mqtt_client.username_pw_set(username, password)
        
        # ROS2 Publishers
        self.pub_task_cmd = self.create_publisher(
            TaskCmd,
            self.get_parameter('task_cmd_topic').value,
            10
        )
        self.pub_emergency = self.create_publisher(
            DriveCmd,
            self.get_parameter('emergency_cmd_topic').value,
            10
        )
        
        # ROS2 Subscribers
        self.sub_task_status = self.create_subscription(
            TaskStatus,
            self.get_parameter('task_status_topic').value,
            self.cb_task_status,
            10
        )
        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self.cb_pose,
            10
        )
        self.sub_driving_state = self.create_subscription(
            DrivingState,
            self.get_parameter('driving_state_topic').value,
            self.cb_driving_state,
            10
        )
        
        # Timers
        loc_period = 1.0 / max(0.1, self.get_parameter('location_publish_rate_hz').value)
        status_period = 1.0 / max(0.1, self.get_parameter('status_publish_rate_hz').value)
        
        self.timer_location = self.create_timer(loc_period, self.publish_location)
        self.timer_status = self.create_timer(status_period, self.publish_status)
        self.timer_reconnect = self.create_timer(5.0, self.check_connection)
        
        # Connect
        self._connect_mqtt()
        
        self.get_logger().info(
            f"mqtt_bridge_node started\n"
            f"  car_id: {self.car_id}\n"
            f"  broker: {self.mqtt_host}:{self.mqtt_port}"
        )
    
    def _connect_mqtt(self):
        """MQTT 브로커 연결"""
        try:
            self.mqtt_client.connect(
                self.mqtt_host,
                self.mqtt_port,
                self.get_parameter('mqtt_keepalive').value
            )
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connect failed: {e}")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT 연결 콜백"""
        if rc == 0:
            self.get_logger().info("MQTT connected")
            with self._lock:
                self._connected = True
            
            # Subscribe
            topics = [
                (f"rc/{self.car_id}/task/cmd", 1),
                (f"rc/{self.car_id}/control/emergency", 2),
                ("rc/broadcast/announce", 0),
            ]
            for topic, qos in topics:
                client.subscribe(topic, qos)
                self.get_logger().info(f"Subscribed: {topic}")
        else:
            self.get_logger().error(f"MQTT connect failed, rc={rc}")
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT 연결 해제 콜백"""
        self.get_logger().warn(f"MQTT disconnected, rc={rc}")
        with self._lock:
            self._connected = False
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신 콜백"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            self.get_logger().debug(f"MQTT recv: {topic}")
            
            if topic == f"rc/{self.car_id}/task/cmd":
                self._handle_task_cmd(data)
            elif topic == f"rc/{self.car_id}/control/emergency":
                self._handle_emergency(data)
            elif topic == "rc/broadcast/announce":
                self._handle_broadcast(data)
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"MQTT message error: {e}")
    
    def _handle_task_cmd(self, data: Dict[str, Any]):
        """태스크 명령 처리"""
        msg = TaskCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = data.get('task_id', '')
        msg.task_type = data.get('task_type', '')
        msg.goal = data.get('goal', '')
        msg.goal_id = data.get('goal_id', -1)
        msg.route_ids = data.get('route_ids', [])
        
        self.pub_task_cmd.publish(msg)
        
        with self._lock:
            self._current_task_id = msg.task_id
        
        self.get_logger().info(
            f"Task received: {msg.task_id}, type={msg.task_type}, "
            f"route={msg.route_ids}, goal={msg.goal_id}"
        )
    
    def _handle_emergency(self, data: Dict[str, Any]):
        """긴급 정지 명령 처리"""
        action = data.get('action', 'stop')
        
        msg = DriveCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "mqtt_emergency"
        
        if action == 'stop':
            msg.enable = False
            msg.vx = 0.0
            msg.vy = 0.0
            msg.wz = 0.0
        elif action == 'resume':
            msg.enable = True
        
        self.pub_emergency.publish(msg)
        self.get_logger().warn(f"Emergency command: {action}")
    
    def _handle_broadcast(self, data: Dict[str, Any]):
        """브로드캐스트 메시지 처리"""
        msg_type = data.get('type', '')
        self.get_logger().info(f"Broadcast: {msg_type}")
    
    # ===== ROS2 Callbacks =====
    
    def cb_task_status(self, msg: TaskStatus):
        """태스크 상태 → MQTT 전송"""
        if not self._connected:
            return
        
        data = {
            'car_id': self.car_id,
            'task_id': msg.task_id,
            'status': msg.status,
            'current_state': msg.current_state,
            'current_marker_id': msg.current_marker_id,
            'progress': msg.progress,
            'message': msg.message,
            'timestamp': time.time()
        }
        
        topic = f"rc/{self.car_id}/task/status"
        self.mqtt_client.publish(topic, json.dumps(data), qos=1)
    
    def cb_pose(self, msg: PoseStamped):
        """위치 저장"""
        with self._lock:
            self._current_pose = msg
    
    def cb_driving_state(self, msg: DrivingState):
        """주행 상태 저장"""
        with self._lock:
            self._current_state = msg
    
    # ===== Periodic Publishers =====
    
    def publish_location(self):
        """위치 정보 주기적 전송"""
        if not self._connected:
            return
        
        with self._lock:
            pose = self._current_pose
        
        if pose is None:
            return
        
        # Quaternion to yaw
        q = pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(siny / (cosy + 1e-9))  # 간단한 근사
        
        import math
        yaw = math.atan2(siny, cosy)
        
        data = {
            'car_id': self.car_id,
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'yaw': yaw,
            'timestamp': time.time()
        }
        
        topic = f"rc/{self.car_id}/location"
        self.mqtt_client.publish(topic, json.dumps(data), qos=0)
    
    def publish_status(self):
        """차량 상태 주기적 전송"""
        if not self._connected:
            return
        
        with self._lock:
            state = self._current_state
            task_id = self._current_task_id
        
        state_str = "IDLE"
        marker_id = -1
        
        if state is not None:
            state_str = state.state
            marker_id = state.observed_marker_id
        
        data = {
            'car_id': self.car_id,
            'state': state_str,
            'battery': self._battery,
            'current_task_id': task_id,
            'current_marker_id': marker_id,
            'timestamp': time.time()
        }
        
        topic = f"rc/{self.car_id}/status"
        self.mqtt_client.publish(topic, json.dumps(data), qos=0)
    
    def check_connection(self):
        """연결 상태 확인 및 재연결"""
        with self._lock:
            connected = self._connected
        
        if not connected:
            self.get_logger().info("Attempting MQTT reconnect...")
            self._connect_mqtt()
    
    def destroy_node(self):
        """종료"""
        if MQTT_AVAILABLE:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
