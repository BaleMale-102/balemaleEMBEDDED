#!/usr/bin/env python3
"""
bridge_node.py - MQTT-ROS2 Bridge for balemale backend

Bridges MQTT communication with ROS2 topics.
Communication developer can modify MQTT logic independently.

MQTT Topics (balemale format):
  Subscribes:
    balemale/robot/{robotId}/cmd - Server commands (type 1: dispatch, 2: reroute, 3: exit)
    balemale/robot/{robotId}/map - Map/node info (type 4)
    balemale/robot/{robotId}/cmd/ack - Server ACK for our requests
  Publishes:
    balemale/robot/{robotId}/request/dispatch - Dispatch request (plate query)  // plate num
    balemale/robot/{robotId}/request/reroute - Reroute request (obstacle)       // 
    balemale/robot/{robotId}/event - Robot events (status updates)              // status 바뀔때마다 발행
    balemale/robot/{robotId}/anomaly - Anomaly detection                        // 아직 구현안됨
    balemale/robot/{robotId}/ack - ACK for server commands                      // 백엔드에서 ack처리안한다고함
    balemale/robot/{robotId}/heartbeat - Heartbeat                              //

ROS2 Topics:
  Subscribes:
    /server/task_status: MissionStatus - Task status to forward as event    // msg 존재함
    /plate/query: PlateQuery - Plate query to forward as dispatch request   // plate 인식 결과
    /mission/state: String - Mission state changes                          // fsm state(controller에서 쓰임)
  Publishes:
    /server/task_cmd: MissionCommand - Task command from server             //  경로 + goal slot
    /plate/response: PlateResponse - Plate verification result              //  
"""

import json
import os
import time
import uuid
import threading
from datetime import datetime
from typing import Any, Dict, Optional, Callable

import yaml
import rclpy
from rclpy.node import Node

import paho.mqtt.client as mqtt

# Standard ROS msgs
from std_msgs.msg import String, Bool

# robot_interfaces - DO NOT CHANGE (used by other nodes)
from robot_interfaces.msg import (
    MissionCommand,
    MissionStatus,
    PlateQuery,
    PlateResponse,
)


# ============================================================================
# Helper functions
# ============================================================================

def now_iso() -> str:
    """Get current timestamp in ISO 8601 format."""
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S")


def now_unix() -> int:
    """Get current Unix timestamp."""
    return int(time.time())


# ============================================================================
# Main Bridge Node
# ============================================================================

class MqttRosBridge(Node):
    """ROS2 to MQTT bridge for balemale backend."""

    def __init__(self):
        super().__init__("server_bridge")

        # ====================================================================
        # ROS Parameters - MQTT Connection
        # ====================================================================
        self.declare_parameter("mqtt.host", "43.202.0.116")
        self.declare_parameter("mqtt.port", 8000)
        self.declare_parameter("mqtt.username", "")
        self.declare_parameter("mqtt.password", "")
        self.declare_parameter("mqtt.client_id", "rc-robot-bridge")
        self.declare_parameter("mqtt.keepalive", 60)
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("simulation", False)

        # ====================================================================
        # ROS Parameters - MQTT Topics (balemale format)
        # ====================================================================
        # Subscribe topics
        self.declare_parameter("mqtt.sub.server_cmd", "balemale/robot/{robotId}/cmd")
        self.declare_parameter("mqtt.sub.server_map", "balemale/robot/{robotId}/map")
        self.declare_parameter("mqtt.sub.server_ack", "balemale/robot/{robotId}/cmd/ack")

        # Publish topics
        self.declare_parameter("mqtt.pub.dispatch_request", "balemale/robot/{robotId}/request/dispatch")
        self.declare_parameter("mqtt.pub.reroute_request", "balemale/robot/{robotId}/request/reroute")
        self.declare_parameter("mqtt.pub.event", "balemale/robot/{robotId}/event")
        self.declare_parameter("mqtt.pub.anomaly", "balemale/robot/{robotId}/anomaly")
        self.declare_parameter("mqtt.pub.ack", "balemale/robot/{robotId}/ack")
        self.declare_parameter("mqtt.pub.heartbeat", "balemale/robot/{robotId}/heartbeat")

        # ====================================================================
        # ROS Parameters - ROS Topics (DO NOT CHANGE - used by other nodes)
        # ====================================================================
        self.declare_parameter("ros.pub.task_cmd", "/server/task_cmd")
        self.declare_parameter("ros.pub.plate_response", "/plate/response")
        self.declare_parameter("ros.sub.task_status", "/server/task_status")
        self.declare_parameter("ros.sub.plate_query", "/plate/query")
        self.declare_parameter("ros.sub.mission_state", "/mission/state")

        # ====================================================================
        # ROS Parameters - Behavior
        # ====================================================================
        self.declare_parameter("heartbeat_hz", 1.0)
        self.declare_parameter("log_throttle_sec", 1.0)

        # ====================================================================
        # ROS Parameters - Map file
        # ====================================================================
        # Default path: config/marker_map.yaml in robot_bringup package
        default_map_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
            "bringup", "robot_bringup", "config", "marker_map.yaml"
        )
        self.declare_parameter("map_file_path", default_map_path)
        self.declare_parameter("ros.pub.map_updated", "/server/map_updated")

        # ====================================================================
        # Load Parameters
        # ====================================================================
        self.mqtt_host = self.get_parameter("mqtt.host").value
        self.mqtt_port = int(self.get_parameter("mqtt.port").value)
        self.mqtt_user = self.get_parameter("mqtt.username").value
        self.mqtt_pass = self.get_parameter("mqtt.password").value
        self.client_id = self.get_parameter("mqtt.client_id").value
        self.keepalive = int(self.get_parameter("mqtt.keepalive").value)
        self.robot_id = str(int(self.get_parameter("robot_id").value))
        self.simulation = bool(self.get_parameter("simulation").value)

        self.hb_hz = float(self.get_parameter("heartbeat_hz").value)
        self.log_thr = float(self.get_parameter("log_throttle_sec").value)

        if self.hb_hz <= 0:
            self.hb_hz = 1.0

        # Format MQTT topics with robot_id
        self.t_sub_server_cmd = self._fmt_topic("mqtt.sub.server_cmd")
        self.t_sub_server_map = self._fmt_topic("mqtt.sub.server_map")
        self.t_sub_server_ack = self._fmt_topic("mqtt.sub.server_ack")
        self.t_pub_dispatch = self._fmt_topic("mqtt.pub.dispatch_request")
        self.t_pub_reroute = self._fmt_topic("mqtt.pub.reroute_request")
        self.t_pub_event = self._fmt_topic("mqtt.pub.event")
        self.t_pub_anomaly = self._fmt_topic("mqtt.pub.anomaly")
        self.t_pub_ack = self._fmt_topic("mqtt.pub.ack")
        self.t_pub_heartbeat = self._fmt_topic("mqtt.pub.heartbeat")

        # ROS topics
        self.ros_pub_task_cmd = self.get_parameter("ros.pub.task_cmd").value
        self.ros_pub_plate_response = self.get_parameter("ros.pub.plate_response").value
        self.ros_sub_task_status = self.get_parameter("ros.sub.task_status").value
        self.ros_sub_plate_query = self.get_parameter("ros.sub.plate_query").value
        self.ros_sub_mission_state = self.get_parameter("ros.sub.mission_state").value

        # Map file
        self.map_file_path = self.get_parameter("map_file_path").value
        self.ros_pub_map_updated = self.get_parameter("ros.pub.map_updated").value

        # ====================================================================
        # State Tracking
        # ====================================================================
        self._current_req_id = ""
        self._current_cmd_id = ""
        self._current_vehicle_id = -1
        self._current_node_id = 0
        self._last_mission_state = ""
        self._last_event_type = ""
        self._last_log_t = 0.0

        # ====================================================================
        # ROS Publishers - DO NOT CHANGE TYPES (used by other nodes)
        # ====================================================================
        self.pub_task_cmd = self.create_publisher(
            MissionCommand, self.ros_pub_task_cmd, 10
        )
        self.pub_plate_response = self.create_publisher(
            PlateResponse, self.ros_pub_plate_response, 10
        )
        self.pub_map_updated = self.create_publisher(
            Bool, self.ros_pub_map_updated, 10
        )

        # ====================================================================
        # ROS Subscribers - DO NOT CHANGE TYPES (used by other nodes)
        # ====================================================================
        self.sub_task_status = self.create_subscription(
            MissionStatus, self.ros_sub_task_status,
            self._cb_task_status, 10
        )
        self.sub_plate_query = self.create_subscription(
            PlateQuery, self.ros_sub_plate_query,
            self._cb_plate_query, 10
        )
        self.sub_mission_state = self.create_subscription(
            String, self.ros_sub_mission_state,
            self._cb_mission_state, 10
        )

        # ====================================================================
        # MQTT Client Setup
        # ====================================================================
        self._mqtt: Optional[mqtt.Client] = None
        self._mqtt_lock = threading.Lock()
        self._connected = False

        if not self.simulation:
            self._setup_mqtt()
            # Start MQTT loop thread
            self._mqtt_thread = threading.Thread(target=self._mqtt_loop, daemon=True)
            self._mqtt_thread.start()
        else:
            self._connected = True
            self.get_logger().info("[SIMULATION] MQTT client not connecting")

        # ====================================================================
        # Heartbeat Timer
        # ====================================================================
        self.timer_hb = self.create_timer(1.0 / self.hb_hz, self._tick_heartbeat)

        # ====================================================================
        # Startup Log
        # ====================================================================
        self.get_logger().info(
            f"MqttRosBridge started. robot_id={self.robot_id} "
            f"mqtt={self.mqtt_host}:{self.mqtt_port} "
            f"simulation={self.simulation}"
        )
        self.get_logger().info(
            f"MQTT sub=[{self.t_sub_server_cmd}, {self.t_sub_server_map}] "
            f"pub=[{self.t_pub_dispatch}, {self.t_pub_event}]"
        )

    # ========================================================================
    # Helper Methods
    # ========================================================================

    def _fmt_topic(self, param_name: str) -> str:
        """Format MQTT topic template with robot_id."""
        tmpl = self.get_parameter(param_name).value
        return tmpl.replace("{robotId}", self.robot_id)

    def _throttle_log(self, msg: str):
        """Log with throttling."""
        t = time.time()
        if t - self._last_log_t >= self.log_thr:
            self._last_log_t = t
            self.get_logger().info(msg)

    def _mqtt_publish(self, topic: str, payload: Dict[str, Any],
                      qos: int = 0, retain: bool = False):
        """Publish to MQTT (thread-safe)."""
        if self.simulation:
            self.get_logger().info(f"[SIM] MQTT publish {topic}: {payload}")
            return

        s = json.dumps(payload, ensure_ascii=False)
        with self._mqtt_lock:
            if self._connected and self._mqtt:
                try:
                    self._mqtt.publish(topic, s, qos=qos, retain=retain)
                except Exception as e:
                    self.get_logger().error(f"MQTT publish error: {e}")

    # ========================================================================
    # MQTT Setup and Callbacks
    # ========================================================================

    def _setup_mqtt(self):
        """Initialize MQTT client."""
        try:
            # paho-mqtt 2.x API
            self._mqtt = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=self.client_id,
                clean_session=True
            )
        except TypeError:
            # paho-mqtt 1.x fallback
            self._mqtt = mqtt.Client(
                client_id=self.client_id,
                clean_session=True
            )

        if self.mqtt_user:
            self._mqtt.username_pw_set(self.mqtt_user, self.mqtt_pass)

        # Last will (robot offline)
        self._mqtt.will_set(
            self.t_pub_heartbeat,
            payload=json.dumps({
                "alive": False,
                "ts": now_iso(),
                "robotId": self.robot_id
            }),
            qos=1,
            retain=True,
        )

        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message = self._on_mqtt_message

    def _mqtt_loop(self):
        """MQTT network loop with auto-reconnect."""
        while rclpy.ok():
            try:
                self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=self.keepalive)
                self._mqtt.loop_forever(retry_first_connection=True)
            except Exception as e:
                self.get_logger().error(f"MQTT loop error: {e}")
                time.sleep(5.0)

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connect callback."""
        self._connected = (rc == 0)
        if self._connected:
            self.get_logger().info(f"MQTT connected rc={rc}")
            # Subscribe
            client.subscribe(self.t_sub_server_cmd, qos=1)
            client.subscribe(self.t_sub_server_map, qos=1)
            client.subscribe(self.t_sub_server_ack, qos=1)
            # Online heartbeat
            self._mqtt_publish(
                self.t_pub_heartbeat,
                {"alive": True, "ts": now_iso(), "robotId": self.robot_id},
                qos=1, retain=True
            )
        else:
            self.get_logger().error(f"MQTT connect failed rc={rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnect callback."""
        self._connected = False
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback - route to handlers."""
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"MQTT RX invalid JSON on {msg.topic}: {e}")
            return

        if msg.topic == self.t_sub_server_cmd:
            self._handle_server_cmd(payload)
        elif msg.topic == self.t_sub_server_map:
            self._handle_server_map(payload)
        elif msg.topic == self.t_sub_server_ack:
            self._handle_server_ack(payload)

    # ========================================================================
    # MQTT -> ROS Handlers (Modify these for communication changes)
    # ========================================================================

    def _handle_server_cmd(self, p: Dict[str, Any]):
        """
        Handle server command from MQTT (type 1, 2, 3).

        Server cmd format:
        {
            "type": "1",  # 1: dispatch, 2: reroute, 3: exit
            "reqId": "...",
            "cmdId": "...",
            "payload": {
                "vehicleId": 1,
                "targetNodeId": 17,
                "path": [0, 1, 5, 17]
            }
        }
        """
        try:
            cmd_type = str(p.get("type", ""))
            req_id = str(p.get("reqId", ""))
            cmd_id = str(p.get("cmdId", ""))
            inner = p.get("payload", {})

            if not isinstance(inner, dict):
                inner = {}

            vehicle_id = int(inner.get("vehicleId", -1))
            target_node_id = int(inner.get("targetNodeId", -1))
            path = inner.get("path", [])

            # Clean path
            clean_path = []
            if isinstance(path, list):
                for x in path:
                    try:
                        clean_path.append(int(x))
                    except (ValueError, TypeError):
                        pass

            # Store for event publishing
            self._current_req_id = req_id
            self._current_cmd_id = cmd_id
            self._current_vehicle_id = vehicle_id

            # Send ACK
            if cmd_id:
                self._publish_ack(cmd_id, True)

            # Route by type
            if cmd_type == "1":
                self.get_logger().info(
                    f"[DISPATCH] req={req_id} vehicle={vehicle_id} "
                    f"target={target_node_id} path={clean_path}"
                )
                self._handle_dispatch_result(req_id, vehicle_id, target_node_id, clean_path)

            elif cmd_type == "2":
                self.get_logger().info(
                    f"[REROUTE] req={req_id} vehicle={vehicle_id} "
                    f"target={target_node_id} path={clean_path}"
                )
                self._handle_reroute_result(req_id, vehicle_id, target_node_id, clean_path)

            elif cmd_type == "3":
                self.get_logger().info(
                    f"[EXIT] vehicle={vehicle_id} target={target_node_id} path={clean_path}"
                )
                self._handle_exit_result(vehicle_id, target_node_id, clean_path)

            else:
                self.get_logger().warn(f"Unknown server cmd type: {cmd_type}")

        except Exception as e:
            self.get_logger().error(f"Server cmd parse error: {e}")

    def _handle_dispatch_result(self, req_id: str, vehicle_id: int,
                                 target_node_id: int, path: list):
        """Handle dispatch result - publish PlateResponse and MissionCommand."""
        # Publish PlateResponse
        resp = PlateResponse()
        resp.header.stamp = self.get_clock().now().to_msg()
        resp.plate_number = ""
        resp.verified = (target_node_id >= 0)
        resp.assigned_slot_id = target_node_id
        resp.waypoint_ids = list(path)
        resp.message = f"Dispatch assigned: slot {target_node_id}" if resp.verified else "Dispatch failed"

        self.pub_plate_response.publish(resp)

        # Publish MissionCommand
        if resp.verified:
            cmd = MissionCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command = "START"
            cmd.waypoint_ids = list(path)
            cmd.final_goal_id = target_node_id
            cmd.task_type = "PARK"
            cmd.task_id = req_id

            self.pub_task_cmd.publish(cmd)
            self.get_logger().info(f"Task command published: PARK to {target_node_id}")

    def _handle_reroute_result(self, req_id: str, vehicle_id: int,
                                target_node_id: int, path: list):
        """Handle reroute result - update path."""
        cmd = MissionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command = "REROUTE"
        cmd.waypoint_ids = list(path)
        cmd.final_goal_id = target_node_id
        cmd.task_type = "REROUTE"
        cmd.task_id = req_id

        self.pub_task_cmd.publish(cmd)
        self.get_logger().info(f"Reroute command published: new path to {target_node_id}")

    def _handle_exit_result(self, vehicle_id: int, target_node_id: int, path: list):
        """Handle exit result."""
        req_id = str(uuid.uuid4())[:8]
        self._current_req_id = req_id
        self._current_vehicle_id = vehicle_id

        cmd = MissionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command = "START"
        cmd.waypoint_ids = list(path)
        cmd.final_goal_id = target_node_id
        cmd.task_type = "EXIT"
        cmd.task_id = req_id

        self.pub_task_cmd.publish(cmd)
        self.get_logger().info(f"Exit command published: drive to {target_node_id}")

    def _handle_server_map(self, p: Dict[str, Any]):
        """
        Handle map data from MQTT (type 4).
        Saves received map to marker_map.yaml file.

        Server format:
        {
            "type": "4",
            "cmdId": "...",
            "payload": {
                "node": [
                    {"nodeId": 1, "nodeCode": "A1", "x": 60.0, "y": 18.0,
                     "nodeType": "ROAD|SLOT", "nodeStatus": "NORMAL"},
                    ...
                ]
            }
        }
        """
        try:
            cmd_id = p.get("cmdId", "")
            inner = p.get("payload", {})
            if not isinstance(inner, dict):
                return

            nodes = inner.get("node", [])
            if not isinstance(nodes, list) or len(nodes) == 0:
                return

            # Send ACK
            if cmd_id:
                self._publish_ack(cmd_id, True)

            # Convert to marker_map.yaml format
            road_markers = {}
            parking_markers = {}

            for node in nodes:
                if not isinstance(node, dict):
                    continue

                node_id = node.get("nodeId", -1)
                if node_id < 0:
                    continue

                x = float(node.get("x", 0.0))
                y = float(node.get("y", 0.0))
                node_type = str(node.get("nodeType", "ROAD")).upper()
                node_code = str(node.get("nodeCode", ""))

                marker_data = {
                    "position": [x, y],
                    "yaw": 1.5708,  # Default: π/2
                }

                if node_type == "SLOT":
                    marker_data["slot_id"] = node_code if node_code else f"S{node_id}"
                    parking_markers[node_id] = marker_data
                else:
                    marker_data["description"] = node_code if node_code else f"Node {node_id}"
                    road_markers[node_id] = marker_data

            # Build yaml structure
            map_data = {
                "unit_scale": 0.01,
                "default_marker_yaw": 1.5708,
                "road_markers": road_markers,
                "parking_markers": parking_markers,
                "turn_map": [],  # Will be calculated dynamically
            }

            # Save to file
            self._save_map_yaml(map_data)

            self.get_logger().info(
                f"Map updated: {len(road_markers)} road + {len(parking_markers)} parking markers"
            )

            # Notify other nodes
            msg = Bool()
            msg.data = True
            self.pub_map_updated.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Map parse error: {e}")

    def _save_map_yaml(self, map_data: Dict[str, Any]):
        """Save map data to yaml file."""
        try:
            # Add header comment
            header = """# Marker Map Configuration
# Auto-generated from server map data
# Unit scale: 0.01 = centimeters to meters
# yaw: marker image "up" direction on map (rad)

"""
            with open(self.map_file_path, 'w', encoding='utf-8') as f:
                f.write(header)
                yaml.dump(map_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

            self.get_logger().info(f"Map saved to: {self.map_file_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

    def _handle_server_ack(self, p: Dict[str, Any]):
        """Handle ACK from server."""
        req_id = p.get("reqId", "")
        is_ack = p.get("isAck", False)
        if is_ack:
            self.get_logger().info(f"Server acknowledged request: {req_id}")

    # ========================================================================
    # ROS -> MQTT Callbacks (Modify these for communication changes)
    # ========================================================================

    def _cb_task_status(self, msg: MissionStatus):
        """
        Handle task status update from mission_manager.
        Forward as event to MQTT only when event type changes.
        """
        if not self._connected:
            return

        # Update current node tracking
        if msg.current_marker_id >= 0:
            self._current_node_id = msg.current_marker_id

        # Map to event type
        event_type = self._map_status_to_event(msg.status, msg.current_state)

        # Only send if changed
        if event_type and event_type != self._last_event_type:
            if self._current_req_id and self._current_vehicle_id >= 0:
                self._publish_event(
                    req_id=self._current_req_id,
                    vehicle_id=self._current_vehicle_id,
                    event_type=event_type
                )
                self.get_logger().info(f"Event changed: {self._last_event_type} -> {event_type}")
            self._last_event_type = event_type

    def _map_status_to_event(self, status: str, state: str) -> str:
        """
        Map MissionStatus to event type for server.

        Event types:
        - WAITING: 대기 중
        - LOADING: 적재 중
        - DRIVING_DESTINATION: 목적지로 이동 중
        - DRIVING_HOME: 집으로 이동 중
        - ESTOP: 멈춤
        - PARKING: 주차중
        - FAILED: 실패
        """
        if status == "COMPLETED":
            return "WAITING"
        elif status == "FAILED":
            return "FAILED"

        state_event_map = {
            "IDLE": "WAITING",
            "WAIT_VEHICLE": "WAITING",
            "LOAD": "LOADING",
            "UNLOAD": "LOADING",
            "DRIVE": "DRIVING_DESTINATION",
            "TURNING": "DRIVING_DESTINATION",
            "ALIGN_TO_MARKER": "DRIVING_DESTINATION",
            "ADVANCE_TO_CENTER": "DRIVING_DESTINATION",
            "STOP_AT_MARKER": "DRIVING_DESTINATION",
            "STOP_BUMP": "DRIVING_DESTINATION",
            "PARK": "PARKING",
            "PARK_DETECT": "PARKING",
            "PARK_ALIGN_MARKER": "PARKING",
            "PARK_FINAL": "PARKING",
            "RETURN_HOME": "DRIVING_HOME",
            "ERROR": "ESTOP",
        }

        return state_event_map.get(state, "")

    def _cb_plate_query(self, msg: PlateQuery):
        """
        Handle plate query from mission_manager.
        Forward as dispatch request to MQTT.
        """
        if not self._connected:
            self.get_logger().warn("Cannot send dispatch request: MQTT not connected")
            return

        req_id = str(uuid.uuid4())[:8]
        self._current_req_id = req_id

        self._publish_dispatch_request(
            req_id=req_id,
            plate=msg.plate_number,
            now_node_id=self._current_node_id,
            target_location="DESTINATION"
        )

        self.get_logger().info(f"Dispatch request sent: plate={msg.plate_number}")

    def _cb_mission_state(self, msg: String):
        """Handle mission state change."""
        if msg.data != self._last_mission_state:
            self._last_mission_state = msg.data

    # ========================================================================
    # MQTT Publish Methods (Modify payloads here for protocol changes)
    # ========================================================================

    def _publish_dispatch_request(self, req_id: str, plate: str,
                                   now_node_id: int, target_location: str):
        """Publish dispatch request to server."""
        payload = {
            "reqId": req_id,
            "plate": plate,
            "nowNodeId": now_node_id,
            "targetLocation": target_location,
            "ts": now_iso()
        }
        self._mqtt_publish(self.t_pub_dispatch, payload, qos=1)

    def _publish_reroute_request(self, req_id: str, vehicle_id: int, now_node_id: int):
        """Publish reroute request to server."""
        payload = {
            "reqId": req_id,
            "vehicleId": vehicle_id,
            "nowNodeId": now_node_id,
            "ts": now_iso()
        }
        self._mqtt_publish(self.t_pub_reroute, payload, qos=1)

    def _publish_event(self, req_id: str, vehicle_id: int, event_type: str):
        """Publish robot event to server."""
        payload = {
            "reqId": req_id,
            "vehicleId": vehicle_id,
            "eventType": event_type,
            "ts": now_iso()
        }
        self._mqtt_publish(self.t_pub_event, payload, qos=1)

    def _publish_anomaly(self, req_id: str, vehicle_id: int, event_type: str,
                         anomaly_type: str, edge_id: int, now_node_id: int, next_node_id: int):
        """Publish anomaly detection to server."""
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
        self._mqtt_publish(self.t_pub_anomaly, payload, qos=1)

    def _publish_ack(self, cmd_id: str, is_ack: bool = True):
        """Publish ACK for server command."""
        payload = {
            "cmdId": cmd_id,
            "isAck": is_ack
        }
        self._mqtt_publish(self.t_pub_ack, payload, qos=1)

    def _tick_heartbeat(self):
        """Periodic heartbeat."""
        if self._connected:
            self._mqtt_publish(
                self.t_pub_heartbeat,
                {"alive": True, "ts": now_iso(), "robotId": self.robot_id},
                qos=0, retain=False
            )

    # ========================================================================
    # Public Methods for External Calls
    # ========================================================================

    def report_anomaly(self, anomaly_type: str, next_node_id: int = -1):
        """Report anomaly to server (HUMAN, OBSTACLE, SYSTEM)."""
        if not self._connected:
            return

        self._publish_anomaly(
            req_id=self._current_req_id,
            vehicle_id=self._current_vehicle_id,
            event_type="ESTOP",
            anomaly_type=anomaly_type,
            edge_id=0,
            now_node_id=self._current_node_id,
            next_node_id=next_node_id if next_node_id >= 0 else self._current_node_id
        )

    def request_reroute(self):
        """Request new route from server due to obstacle."""
        if not self._connected:
            return

        self._publish_reroute_request(
            req_id=self._current_req_id,
            vehicle_id=self._current_vehicle_id,
            now_node_id=self._current_node_id
        )

    # ========================================================================
    # Cleanup
    # ========================================================================

    def destroy_node(self):
        """Clean up on shutdown."""
        if self._mqtt is not None:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
            self._mqtt = None
        super().destroy_node()


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MqttRosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
