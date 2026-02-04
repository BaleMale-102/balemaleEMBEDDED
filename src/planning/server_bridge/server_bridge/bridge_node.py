#!/usr/bin/env python3
"""
bridge_node.py - Server Bridge Node

Bridges MQTT communication with ROS2 topics using balemale backend format.

MQTT Topics (balemale format):
  Subscribes:
    balemale/robot/{robotId}/cmd - Server commands (type 1: dispatch, 2: reroute, 3: exit)
    balemale/robot/{robotId}/map - Map/node info (type 4)
    balemale/robot/{robotId}/cmd/ack - Server ACK for our requests
  Publishes:
    balemale/robot/{robotId}/request/dispatch - Dispatch request (plate query)
    balemale/robot/{robotId}/request/reroute - Reroute request (obstacle)
    balemale/robot/{robotId}/event - Robot events (status updates)
    balemale/robot/{robotId}/anomaly - Anomaly detection
    balemale/robot/{robotId}/ack - ACK for server commands

ROS2 Topics:
  Subscribes:
    /server/task_status: MissionStatus - Task status to forward as event
    /plate/query: PlateQuery - Plate query to forward as dispatch request
    /mission/state: String - Mission state changes
  Publishes:
    /server/task_cmd: MissionCommand - Task command from server
    /plate/response: PlateResponse - Plate verification result
"""

import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_interfaces.msg import (
    MissionCommand, MissionStatus,
    PlateQuery, PlateResponse
)

from .mqtt_client import MQTTClient, MQTTConfig


class ServerBridgeNode(Node):
    """ROS2 to MQTT bridge for balemale backend."""

    def __init__(self):
        super().__init__('server_bridge_node')

        # Parameters
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 8000)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('simulation', False)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('status_rate', 2.0)

        # Load parameters
        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        mqtt_username = self.get_parameter('mqtt_username').value
        mqtt_password = self.get_parameter('mqtt_password').value
        robot_id = self.get_parameter('robot_id').value
        simulation = self.get_parameter('simulation').value
        heartbeat_rate = self.get_parameter('heartbeat_rate').value

        # MQTT configuration
        config = MQTTConfig(
            host=mqtt_host,
            port=mqtt_port,
            username=mqtt_username,
            password=mqtt_password,
            robot_id=robot_id
        )

        self.robot_id = robot_id

        # State tracking
        self._current_req_id = ""
        self._current_cmd_id = ""
        self._current_vehicle_id = -1
        self._current_node_id = 0
        self._next_node_id = -1
        self._last_mission_state = ""

        # MQTT client
        self.mqtt = MQTTClient(config, simulation, logger=self.get_logger())
        self.mqtt.set_connect_callback(self._mqtt_connect_callback)
        self.mqtt.set_server_cmd_callback(self._mqtt_server_cmd_callback)
        self.mqtt.set_map_callback(self._mqtt_map_callback)
        self.mqtt.set_ack_callback(self._mqtt_ack_callback)

        # Publishers
        self.pub_task_cmd = self.create_publisher(
            MissionCommand, '/server/task_cmd', 10
        )
        self.pub_plate_response = self.create_publisher(
            PlateResponse, '/plate/response', 10
        )

        # Subscribers
        self.sub_task_status = self.create_subscription(
            MissionStatus, '/server/task_status',
            self._task_status_callback, 10
        )
        self.sub_plate_query = self.create_subscription(
            PlateQuery, '/plate/query',
            self._plate_query_callback, 10
        )

        self.sub_mission_state = self.create_subscription(
            String, '/mission/state',
            self._mission_state_callback, 10
        )

        # Timers
        if heartbeat_rate > 0:
            self.heartbeat_timer = self.create_timer(
                1.0 / heartbeat_rate, self._heartbeat_callback
            )

        # Connect to MQTT
        if self.mqtt.connect():
            self.get_logger().info(
                f'Connecting to MQTT broker at {mqtt_host}:{mqtt_port}'
            )
        else:
            self.get_logger().error('MQTT connection failed')

        self.get_logger().info(
            f'ServerBridgeNode started (robot_id={robot_id})'
        )

    def _mqtt_connect_callback(self, connected: bool):
        """Handle MQTT connection status."""
        if connected:
            self.get_logger().info('MQTT connected')
        else:
            self.get_logger().warn('MQTT disconnected')

    def _mqtt_ack_callback(self, payload: dict):
        """Handle ACK from server for our requests."""
        req_id = payload.get("reqId", "")
        is_ack = payload.get("isAck", False)
        if is_ack:
            self.get_logger().info(f'Server acknowledged request: {req_id}')

    def _mqtt_server_cmd_callback(self, payload: dict):
        """
        Handle server command from MQTT.

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
            cmd_type = str(payload.get("type", ""))
            req_id = str(payload.get("reqId", ""))
            cmd_id = str(payload.get("cmdId", ""))
            inner = payload.get("payload", {})

            if not isinstance(inner, dict):
                inner = {}

            vehicle_id = int(inner.get("vehicleId", -1))
            target_node_id = int(inner.get("targetNodeId", -1))
            path = inner.get("path", [])

            # Clean path to integers
            clean_path = []
            if isinstance(path, list):
                for p in path:
                    try:
                        clean_path.append(int(p))
                    except (ValueError, TypeError):
                        pass

            # Store for event publishing
            self._current_req_id = req_id
            self._current_cmd_id = cmd_id
            self._current_vehicle_id = vehicle_id

            # Send ACK for the command
            if cmd_id:
                self.mqtt.publish_ack(cmd_id, True)

            if cmd_type == "1":
                # Dispatch result - respond to plate query
                self.get_logger().info(
                    f'[DISPATCH] req={req_id} vehicle={vehicle_id} '
                    f'target={target_node_id} path={clean_path}'
                )
                self._handle_dispatch_result(req_id, vehicle_id, target_node_id, clean_path)

            elif cmd_type == "2":
                # Reroute result
                self.get_logger().info(
                    f'[REROUTE] req={req_id} vehicle={vehicle_id} '
                    f'target={target_node_id} path={clean_path}'
                )
                self._handle_reroute_result(req_id, vehicle_id, target_node_id, clean_path)

            elif cmd_type == "3":
                # Exit result (출차)
                self.get_logger().info(
                    f'[EXIT] vehicle={vehicle_id} target={target_node_id} path={clean_path}'
                )
                self._handle_exit_result(vehicle_id, target_node_id, clean_path)

            else:
                self.get_logger().warn(f'Unknown server cmd type: {cmd_type}')

        except Exception as e:
            self.get_logger().error(f'Server cmd parse error: {e}')

    def _handle_dispatch_result(self, req_id: str, vehicle_id: int,
                                 target_node_id: int, path: list):
        """Handle dispatch result - publish PlateResponse and MissionCommand."""
        # Publish PlateResponse
        resp = PlateResponse()
        resp.header.stamp = self.get_clock().now().to_msg()
        resp.plate_number = ""  # Server doesn't echo back plate
        resp.verified = (target_node_id >= 0)
        resp.assigned_slot_id = target_node_id
        resp.waypoint_ids = path[:-1] if len(path) > 1 else []  # All except final
        resp.message = f"Dispatch assigned: slot {target_node_id}" if resp.verified else "Dispatch failed"

        self.pub_plate_response.publish(resp)

        # Publish MissionCommand to start driving
        if resp.verified:
            cmd = MissionCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command = "START"
            cmd.waypoint_ids = path[:-1] if len(path) > 1 else []
            cmd.final_goal_id = target_node_id
            cmd.task_type = "PARK"
            cmd.task_id = req_id

            self.pub_task_cmd.publish(cmd)
            self.get_logger().info(f'Task command published: PARK to {target_node_id}')

    def _handle_reroute_result(self, req_id: str, vehicle_id: int,
                                target_node_id: int, path: list):
        """Handle reroute result - update path."""
        cmd = MissionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command = "REROUTE"
        cmd.waypoint_ids = path[:-1] if len(path) > 1 else []
        cmd.final_goal_id = target_node_id
        cmd.task_type = "REROUTE"
        cmd.task_id = req_id

        self.pub_task_cmd.publish(cmd)
        self.get_logger().info(f'Reroute command published: new path to {target_node_id}')

    def _handle_exit_result(self, vehicle_id: int, target_node_id: int, path: list):
        """Handle exit (출차) result."""
        req_id = str(uuid.uuid4())[:8]
        self._current_req_id = req_id
        self._current_vehicle_id = vehicle_id

        cmd = MissionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command = "START"
        cmd.waypoint_ids = path[:-1] if len(path) > 1 else []
        cmd.final_goal_id = target_node_id
        cmd.task_type = "EXIT"
        cmd.task_id = req_id

        self.pub_task_cmd.publish(cmd)
        self.get_logger().info(f'Exit command published: drive to {target_node_id}')

    def _mqtt_map_callback(self, payload: dict):
        """
        Handle map data from MQTT (type 4).

        Format:
        {
            "type": "4",
            "cmdId": "...",
            "payload": {
                "node": [
                    {"nodeId": 1, "nodeCode": "A1", "x": 0.5, "y": 0.5, "nodeType": "SLOT", "nodeStatus": "NORMAL"},
                    ...
                ]
            }
        }
        """
        try:
            cmd_id = payload.get("cmdId", "")
            inner = payload.get("payload", {})
            if not isinstance(inner, dict):
                return

            nodes = inner.get("node", [])
            if not isinstance(nodes, list):
                return

            # Send ACK
            if cmd_id:
                self.mqtt.publish_ack(cmd_id, True)

            self.get_logger().info(f'Received map with {len(nodes)} nodes')
            # TODO: Publish to /server/map_nodes if needed

        except Exception as e:
            self.get_logger().error(f'Map parse error: {e}')

    def _task_status_callback(self, msg):
        """
        Handle task status update from mission_manager.
        Forward as event to MQTT.
        """
        if not self.mqtt.is_connected:
            return

        # Update current node tracking (only if valid, keep previous value if -1)
        if msg.current_marker_id >= 0:
            self._current_node_id = msg.current_marker_id

        # Map MissionStatus to event type
        event_type = self._map_status_to_event(msg.status, msg.current_state)

        # Only send event if we have valid req_id and vehicle_id (from server response)
        if event_type and self._current_req_id and self._current_vehicle_id >= 0:
            self.mqtt.publish_event(
                req_id=self._current_req_id,
                vehicle_id=self._current_vehicle_id,
                event_type=event_type
            )

    def _map_status_to_event(self, status: str, state: str) -> str:
        """Map MissionStatus to event type for server."""
        # Status-based events
        if status == "COMPLETED":
            return "PARKED"  # Mission completed = parked
        elif status == "FAILED":
            return "ESTOP"

        # State-based events (match server protocol)
        state_event_map = {
            "LOAD": "LOADING",
            "UNLOAD": "LOADING",  # Unload is also a loading operation
            "DRIVE": "MOVING",
            "TURNING": "MOVING",
            "ALIGN_TO_MARKER": "MOVING",
            "ADVANCE_TO_CENTER": "MOVING",
            "PARK": "MOVING",
            "PARK_DETECT": "MOVING",
            "PARK_ALIGN_MARKER": "MOVING",
            "PARK_FINAL": "PARKED",
            "RETURN_HOME": "MOVING",
            "WAIT_VEHICLE": "PARKED",  # Waiting at home = parked state
            "ERROR": "ESTOP",
        }

        return state_event_map.get(state, "")

    def _plate_query_callback(self, msg):
        """
        Handle plate query from mission_manager.
        Forward as dispatch request to MQTT.
        """
        if not self.mqtt.is_connected:
            self.get_logger().warn('Cannot send dispatch request: MQTT not connected')
            return

        # Generate request ID if not provided
        req_id = str(uuid.uuid4())[:8]
        self._current_req_id = req_id

        self.mqtt.publish_dispatch_request(
            req_id=req_id,
            plate=msg.plate_number,
            now_node_id=self._current_node_id,
            target_location="DESTINATION"
        )

        self.get_logger().info(f'Dispatch request sent: plate={msg.plate_number}')

    def _mission_state_callback(self, msg: String):
        """Handle mission state change."""
        if msg.data != self._last_mission_state:
            self._last_mission_state = msg.data
            # Could publish state change event if needed

    def _heartbeat_callback(self):
        """Periodic heartbeat."""
        if self.mqtt.is_connected:
            self.mqtt.publish_heartbeat()

    # Public methods for anomaly reporting (can be called from other nodes via service)
    def report_anomaly(self, anomaly_type: str, next_node_id: int = -1):
        """
        Report anomaly to server.

        Args:
            anomaly_type: HUMAN, OBSTACLE, SYSTEM
            next_node_id: Next node ID if known
        """
        if not self.mqtt.is_connected:
            return

        self.mqtt.publish_anomaly(
            req_id=self._current_req_id,
            vehicle_id=self._current_vehicle_id,
            event_type="ESTOP",
            anomaly_type=anomaly_type,
            edge_id=0,  # TODO: Calculate from current/next node
            now_node_id=self._current_node_id,
            next_node_id=next_node_id if next_node_id >= 0 else self._current_node_id
        )

    def request_reroute(self):
        """Request new route from server due to obstacle."""
        if not self.mqtt.is_connected:
            return

        self.mqtt.publish_reroute_request(
            req_id=self._current_req_id,
            vehicle_id=self._current_vehicle_id,
            now_node_id=self._current_node_id
        )

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
