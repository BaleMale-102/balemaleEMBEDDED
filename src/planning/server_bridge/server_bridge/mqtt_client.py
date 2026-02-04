#!/usr/bin/env python3
"""
mqtt_client.py - MQTT Client Wrapper

Thread-safe MQTT client for server communication.
MQTT topics follow balemale backend format:
  - Subscribe: balemale/robot/{robotId}/cmd, balemale/robot/{robotId}/map, balemale/robot/{robotId}/cmd/ack
  - Publish: balemale/robot/{robotId}/request/dispatch, balemale/robot/{robotId}/request/reroute,
             balemale/robot/{robotId}/event, balemale/robot/{robotId}/anomaly, balemale/robot/{robotId}/ack
"""

import json
import threading
import time
from datetime import datetime
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass

try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False


@dataclass
class MQTTConfig:
    """MQTT connection configuration."""
    host: str = 'localhost'
    port: int = 8000
    username: str = ''
    password: str = ''
    robot_id: int = 1
    keepalive: int = 60
    reconnect_delay: float = 5.0


def now_iso() -> str:
    """Get current timestamp in ISO 8601 format."""
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S")


def now_unix() -> int:
    """Get current Unix timestamp."""
    return int(time.time())


class MQTTClient:
    """Thread-safe MQTT client for balemale backend."""

    def __init__(self, config: MQTTConfig, simulation: bool = False, logger=None):
        """
        Initialize MQTT client.

        Args:
            config: MQTT configuration
            simulation: If True, operate without actual connection
            logger: ROS logger instance
        """
        self.config = config
        self.simulation = simulation
        self._logger = logger

        self._client: Optional[mqtt.Client] = None
        self._connected = False
        self._lock = threading.Lock()

        # Callbacks for specific message types
        self._on_connect_cb: Optional[Callable[[bool], None]] = None
        self._on_server_cmd_cb: Optional[Callable[[dict], None]] = None
        self._on_map_cb: Optional[Callable[[dict], None]] = None
        self._on_ack_cb: Optional[Callable[[dict], None]] = None

        # Robot ID for topic formatting
        self.robot_id = str(config.robot_id)

        # Topic templates (balemale backend format)
        # Subscribe topics
        self._topic_sub_cmd = f"balemale/robot/{self.robot_id}/cmd"
        self._topic_sub_map = f"balemale/robot/{self.robot_id}/map"
        self._topic_sub_cmd_ack = f"balemale/robot/{self.robot_id}/cmd/ack"

        # Publish topics
        self._topic_pub_dispatch = f"balemale/robot/{self.robot_id}/request/dispatch"
        self._topic_pub_reroute = f"balemale/robot/{self.robot_id}/request/reroute"
        self._topic_pub_event = f"balemale/robot/{self.robot_id}/event"
        self._topic_pub_anomaly = f"balemale/robot/{self.robot_id}/anomaly"
        self._topic_pub_ack = f"balemale/robot/{self.robot_id}/ack"
        self._topic_pub_heartbeat = f"balemale/robot/{self.robot_id}/heartbeat"

    def _log_info(self, msg: str):
        if self._logger:
            self._logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def _log_warn(self, msg: str):
        if self._logger:
            self._logger.warn(msg)
        else:
            print(f"[WARN] {msg}")

    def _log_error(self, msg: str):
        if self._logger:
            self._logger.error(msg)
        else:
            print(f"[ERROR] {msg}")

    def connect(self) -> bool:
        """Connect to MQTT broker."""
        if self.simulation:
            self._connected = True
            if self._on_connect_cb:
                self._on_connect_cb(True)
            return True

        if not HAS_MQTT:
            self._log_error("paho-mqtt not installed")
            return False

        try:
            client_id = f"rc-robot-{self.robot_id}"
            # paho-mqtt 2.x API
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=client_id,
                clean_session=True
            )

            if self.config.username:
                self._client.username_pw_set(
                    self.config.username,
                    self.config.password
                )

            # Last will (robot offline notification)
            self._client.will_set(
                self._topic_pub_heartbeat,
                payload=json.dumps({
                    "alive": False,
                    "ts": now_iso(),
                    "robotId": self.robot_id
                }),
                qos=1,
                retain=True
            )

            self._client.on_connect = self._mqtt_on_connect
            self._client.on_disconnect = self._mqtt_on_disconnect
            self._client.on_message = self._mqtt_on_message

            self._client.connect(
                self.config.host,
                self.config.port,
                self.config.keepalive
            )

            # Start network loop in background
            self._client.loop_start()

            return True

        except Exception as e:
            self._log_error(f"MQTT connect error: {e}")
            return False

    def disconnect(self):
        """Disconnect from broker."""
        if self._client is not None:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None
        self._connected = False

    def set_connect_callback(self, callback: Callable[[bool], None]):
        """Set connection status callback."""
        self._on_connect_cb = callback

    def set_server_cmd_callback(self, callback: Callable[[dict], None]):
        """Set callback for server commands (type 1,2,3)."""
        self._on_server_cmd_cb = callback

    def set_map_callback(self, callback: Callable[[dict], None]):
        """Set callback for map data (type 4)."""
        self._on_map_cb = callback

    def set_ack_callback(self, callback: Callable[[dict], None]):
        """Set callback for ACK from server."""
        self._on_ack_cb = callback

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _publish(self, topic: str, payload: dict, qos: int = 0, retain: bool = False):
        """Internal publish method."""
        if self.simulation:
            self._log_info(f"[SIM] Publish {topic}: {payload}")
            return

        with self._lock:
            if self._client and self._connected:
                try:
                    json_payload = json.dumps(payload, ensure_ascii=False)
                    self._client.publish(topic, json_payload, qos=qos, retain=retain)
                except Exception as e:
                    self._log_error(f"Publish error: {e}")

    def publish_dispatch_request(self, req_id: str, plate: str, now_node_id: int,
                                  target_location: str = "DESTINATION"):
        """
        Publish dispatch request to server.

        Args:
            req_id: Unique request ID
            plate: License plate number
            now_node_id: Current node/marker ID
            target_location: Target location type (default: DESTINATION)
        """
        payload = {
            "reqId": req_id,
            "plate": plate,
            "nowNodeId": now_node_id,
            "targetLocation": target_location,
            "ts": now_iso()
        }
        self._publish(self._topic_pub_dispatch, payload, qos=1)
        self._log_info(f"Dispatch request sent: plate={plate}, node={now_node_id}")

    def publish_reroute_request(self, req_id: str, vehicle_id: int, now_node_id: int):
        """
        Publish reroute request to server (obstacle avoidance).

        Args:
            req_id: Request ID
            vehicle_id: Vehicle ID from server
            now_node_id: Current node ID
        """
        payload = {
            "reqId": req_id,
            "vehicleId": vehicle_id,
            "nowNodeId": now_node_id,
            "ts": now_iso()
        }
        self._publish(self._topic_pub_reroute, payload, qos=1)
        self._log_info(f"Reroute request sent: vehicle={vehicle_id}, node={now_node_id}")

    def publish_event(self, req_id: str, vehicle_id: int, event_type: str):
        """
        Publish robot event to server.

        Args:
            req_id: Request ID this event belongs to
            vehicle_id: Vehicle ID (from server)
            event_type: Event type (LOADING, MOVING, PARKED, ESTOP)
        """
        payload = {
            "reqId": req_id,
            "vehicleId": vehicle_id,
            "eventType": event_type,
            "ts": now_iso()
        }
        self._publish(self._topic_pub_event, payload, qos=1)
        self._log_info(f"Event sent: {event_type} (req={req_id})")

    def publish_anomaly(self, req_id: str, vehicle_id: int, event_type: str,
                        anomaly_type: str, edge_id: int, now_node_id: int, next_node_id: int):
        """
        Publish anomaly detection to server.

        Args:
            req_id: Request ID
            vehicle_id: Vehicle ID
            event_type: Event type (ESTOP)
            anomaly_type: Anomaly type (HUMAN, OBSTACLE, SYSTEM)
            edge_id: Current edge ID
            now_node_id: Current node ID
            next_node_id: Next node ID
        """
        payload = {
            "reqId": req_id,
            "vehicleId": vehicle_id,
            "eventType": event_type,
            "anomalyType": anomaly_type,
            "edgeId": edge_id,
            "nowNodeId": now_node_id,
            "nextNodeId": next_node_id,
            "ts": now_iso()
        }
        self._publish(self._topic_pub_anomaly, payload, qos=1)
        self._log_info(f"Anomaly sent: {anomaly_type} at node {now_node_id}")

    def publish_ack(self, cmd_id: str, is_ack: bool = True):
        """
        Publish ACK for server command.

        Args:
            cmd_id: Server command ID (UUID)
            is_ack: Acknowledgment status
        """
        payload = {
            "cmdId": cmd_id,
            "isAck": is_ack
        }
        self._publish(self._topic_pub_ack, payload, qos=1)
        self._log_info(f"ACK sent: cmdId={cmd_id}")

    def publish_heartbeat(self):
        """Publish heartbeat to server."""
        payload = {
            "alive": True,
            "ts": now_iso(),
            "robotId": self.robot_id
        }
        self._publish(self._topic_pub_heartbeat, payload, qos=0, retain=False)

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        """MQTT connect callback."""
        self._connected = (rc == 0)

        if self._connected:
            self._log_info(f"MQTT connected to {self.config.host}:{self.config.port}")

            # Subscribe to server topics
            client.subscribe(self._topic_sub_cmd, qos=1)
            client.subscribe(self._topic_sub_map, qos=1)
            client.subscribe(self._topic_sub_cmd_ack, qos=1)
            self._log_info(f"Subscribed: {self._topic_sub_cmd}, {self._topic_sub_map}, {self._topic_sub_cmd_ack}")

            # Send online heartbeat
            self.publish_heartbeat()
        else:
            self._log_error(f"MQTT connection failed rc={rc}")

        if self._on_connect_cb:
            self._on_connect_cb(self._connected)

    def _mqtt_on_disconnect(self, client, userdata, rc):
        """MQTT disconnect callback."""
        self._connected = False
        self._log_warn(f"MQTT disconnected rc={rc}")

        if self._on_connect_cb:
            self._on_connect_cb(False)

    def _mqtt_on_message(self, client, userdata, msg):
        """MQTT message callback."""
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except json.JSONDecodeError as e:
            self._log_error(f"Invalid JSON on {msg.topic}: {e}")
            return

        topic = msg.topic

        if topic == self._topic_sub_cmd:
            # Server command (type 1, 2, 3)
            if self._on_server_cmd_cb:
                self._on_server_cmd_cb(payload)

        elif topic == self._topic_sub_map:
            # Map data (type 4)
            if self._on_map_cb:
                self._on_map_cb(payload)

        elif topic == self._topic_sub_cmd_ack:
            # ACK from server for our request
            if self._on_ack_cb:
                self._on_ack_cb(payload)
            self._log_info(f"Server ACK received: reqId={payload.get('reqId')}")
