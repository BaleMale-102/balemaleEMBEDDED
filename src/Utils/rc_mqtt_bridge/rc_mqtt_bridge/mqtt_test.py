#!/usr/bin/env python3
import json
import time
import threading
from typing import Any, Dict

import rclpy
from rclpy.node import Node

import paho.mqtt.client as mqtt
from std_msgs.msg import String


def now_unix() -> int:
    return int(time.time())


class MqttRosBridgeRaw(Node):

    def __init__(self):
        super().__init__("mqtt_ros_bridge_raw")

        # Params
        self.declare_parameter("mqtt.host", "127.0.0.1")
        self.declare_parameter("mqtt.port", 1883)
        self.declare_parameter("mqtt.sub.task_cmd", "rc/server/task_cmd")
        self.declare_parameter("mqtt.pub.heartbeat", "rc/robot/heartbeat")
        self.declare_parameter("ros.pub.task_cmd_raw", "/server/task_cmd_raw")
        self.declare_parameter("heartbeat_hz", 1.0)

        self.mqtt_host = self.get_parameter("mqtt.host").value
        self.mqtt_port = int(self.get_parameter("mqtt.port").value)
        self.t_sub_task_cmd = self.get_parameter("mqtt.sub.task_cmd").value
        self.t_pub_heartbeat = self.get_parameter("mqtt.pub.heartbeat").value
        self.ros_pub_task_cmd_raw = self.get_parameter("ros.pub.task_cmd_raw").value
        self.hz = float(self.get_parameter("heartbeat_hz").value) or 1.0

        # ROS publisher
        self.pub_raw = self.create_publisher(String, self.ros_pub_task_cmd_raw, 10)

        # MQTT client
        self._mqtt = mqtt.Client(client_id="rc-robot-bridge-raw", clean_session=True)
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt.on_disconnect = self._on_disconnect

        self._connected = False
        self._lock = threading.Lock()

        # MQTT loop thread
        threading.Thread(target=self._mqtt_loop, daemon=True).start()

        # heartbeat timer
        self.create_timer(1.0 / self.hz, self._tick_heartbeat)

        self.get_logger().info(
            f"[RAW] ready mqtt={self.mqtt_host}:{self.mqtt_port} "
            f"sub={self.t_sub_task_cmd} pub_ros={self.ros_pub_task_cmd_raw}"
        )

    def _mqtt_loop(self):
        while rclpy.ok():
            try:
                self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
                self._mqtt.loop_forever(retry_first_connection=True)
            except Exception as e:
                self.get_logger().error(f"MQTT loop error: {e}")
                time.sleep(1.0)

    def _on_connect(self, client, userdata, flags, rc):
        self._connected = True
        self.get_logger().info(f"MQTT connected rc={rc}")
        client.subscribe(self.t_sub_task_cmd, qos=0)

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def _on_message(self, client, userdata, msg):
        raw = msg.payload.decode("utf-8", errors="replace")
        try:
            obj = json.loads(raw)
            raw = json.dumps(obj, ensure_ascii=False)
        except Exception:
            pass

        ros_msg = String()
        ros_msg.data = raw
        self.pub_raw.publish(ros_msg)

        self.get_logger().info(f"[RAW] MQTT->ROS {msg.topic}: {raw}")

    def _tick_heartbeat(self):
        if not self._connected:
            return
        payload = json.dumps({"alive": True, "ts": now_unix(), "bridge": "raw"}, ensure_ascii=False)
        with self._lock:
            self._mqtt.publish(self.t_pub_heartbeat, payload, qos=0, retain=False)


def main():
    rclpy.init()
    node = MqttRosBridgeRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

