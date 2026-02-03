#!/usr/bin/env python3
"""
mqtt_client.py - MQTT Client Wrapper

Thread-safe MQTT client for server communication.
"""

import json
import threading
import time
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
    port: int = 1883
    username: str = ''
    password: str = ''
    client_id: str = 'rc_car'
    keepalive: int = 60
    reconnect_delay: float = 5.0


class MQTTClient:
    """Thread-safe MQTT client."""

    def __init__(self, config: MQTTConfig, simulation: bool = False):
        """
        Initialize MQTT client.

        Args:
            config: MQTT configuration
            simulation: If True, operate without actual connection
        """
        self.config = config
        self.simulation = simulation

        self._client: Optional[mqtt.Client] = None
        self._connected = False
        self._lock = threading.Lock()

        # Callbacks
        self._message_callbacks: Dict[str, Callable[[str, dict], None]] = {}
        self._on_connect: Optional[Callable[[bool], None]] = None

        # Topic prefixes
        self._pub_prefix = f'car/{config.client_id}/'
        self._sub_prefix = f'server/{config.client_id}/'

    def connect(self) -> bool:
        """Connect to MQTT broker."""
        if self.simulation:
            self._connected = True
            if self._on_connect:
                self._on_connect(True)
            return True

        if not HAS_MQTT:
            return False

        try:
            self._client = mqtt.Client(client_id=self.config.client_id)

            if self.config.username:
                self._client.username_pw_set(
                    self.config.username,
                    self.config.password
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
            print(f'MQTT connect error: {e}')
            return False

    def disconnect(self):
        """Disconnect from broker."""
        if self._client is not None:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None
        self._connected = False

    def subscribe(self, topic: str, callback: Callable[[str, dict], None]):
        """
        Subscribe to a topic.

        Args:
            topic: Topic name (without prefix)
            callback: Function called with (topic, payload_dict)
        """
        full_topic = self._sub_prefix + topic
        self._message_callbacks[full_topic] = callback

        if self._client and self._connected:
            self._client.subscribe(full_topic)

    def publish(self, topic: str, payload: dict, retain: bool = False):
        """
        Publish message to topic.

        Args:
            topic: Topic name (without prefix)
            payload: Dictionary to send as JSON
            retain: Whether to retain message
        """
        if self.simulation:
            return

        full_topic = self._pub_prefix + topic

        with self._lock:
            if self._client and self._connected:
                try:
                    json_payload = json.dumps(payload)
                    self._client.publish(full_topic, json_payload, retain=retain)
                except Exception:
                    pass

    def publish_status(self, status: str, data: dict = None):
        """Publish robot status."""
        payload = {
            'status': status,
            'timestamp': time.time(),
        }
        if data:
            payload.update(data)
        self.publish('status', payload)

    def publish_pose(self, x: float, y: float, yaw: float):
        """Publish robot pose."""
        self.publish('pose', {
            'x': x,
            'y': y,
            'yaw': yaw,
            'timestamp': time.time()
        })

    def set_connect_callback(self, callback: Callable[[bool], None]):
        """Set connection status callback."""
        self._on_connect = callback

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        """MQTT connect callback."""
        self._connected = (rc == 0)

        if self._connected:
            # Subscribe to registered topics
            for topic in self._message_callbacks:
                client.subscribe(topic)

        if self._on_connect:
            self._on_connect(self._connected)

    def _mqtt_on_disconnect(self, client, userdata, rc):
        """MQTT disconnect callback."""
        self._connected = False
        if self._on_connect:
            self._on_connect(False)

    def _mqtt_on_message(self, client, userdata, msg):
        """MQTT message callback."""
        topic = msg.topic
        callback = self._message_callbacks.get(topic)

        if callback:
            try:
                payload = json.loads(msg.payload.decode('utf-8'))
                callback(topic, payload)
            except json.JSONDecodeError:
                pass
            except Exception:
                pass
