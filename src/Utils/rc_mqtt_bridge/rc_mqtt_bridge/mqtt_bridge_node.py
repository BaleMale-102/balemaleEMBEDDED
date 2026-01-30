#!/usr/bin/env python3
import json
import time
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

import paho.mqtt.client as mqtt

# ROS msgs (프로젝트에 맞게 존재하는 것만 쓰면 됨)
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from rc_interfaces.msg import TaskCmd, TaskStatus, DriveCmd, ServerCmd, DispatchRequest
#from rc_interfaces.msg import TaskCmd, TaskStatus, DriveCmd 원본


def now_unix() -> int:
    return int(time.time())


class MqttRosBridge(Node):
    def __init__(self):
        super().__init__("mqtt_ros_bridge")

        # ---------- ROS Params ----------
        self.declare_parameter("mqtt.host", "127.0.0.1")
        self.declare_parameter("mqtt.port", 1883)
        self.declare_parameter("mqtt.username", "")
        self.declare_parameter("mqtt.password", "")
        self.declare_parameter("mqtt.client_id", "rc-robot-bridge")
        self.declare_parameter("mqtt.keepalive", 30)
        self.declare_parameter("mqtt.tls", False)  # 필요하면 확장

        # MQTT topics
        self.declare_parameter("mqtt.sub.task_cmd", "rc/server/task_cmd")
        self.declare_parameter("mqtt.sub.drive_cmd", "rc/server/drive_cmd")  # 선택
        self.declare_parameter("mqtt.pub.task_status", "rc/robot/task_status")
        self.declare_parameter("mqtt.pub.mission_state", "rc/robot/mission_state")
        self.declare_parameter("mqtt.pub.pose", "rc/robot/pose")
        self.declare_parameter("mqtt.pub.heartbeat", "rc/robot/heartbeat")
        self.declare_parameter("mqtt.pub.dispatch_request", "balemale/robot/{robotId}/request/dispatch")


        # ROS topics
        self.declare_parameter("ros.pub.task_cmd", "/server/task_cmd")
        self.declare_parameter("ros.pub.drive_cmd", "/control/drive_cmd_safe") 

        self.declare_parameter("ros.sub.task_status", "/server/task_status")
        self.declare_parameter("ros.sub.mission_state", "/mission/state")
        self.declare_parameter("ros.sub.pose", "/localization/pose")
        self.declare_parameter("ros.sub.dispatch_request", "/robot/request/dispatch")

        
        # balemale topics (server -> robot cmd)
        self.declare_parameter("robot_id", "1")
        self.declare_parameter("mqtt.sub.server_cmd", "balemale/robot/{robotId}/cmd")

        # ROS topic for server cmd
        self.declare_parameter("ros.pub.server_cmd", "/server/cmd")


        # Behavior
        self.declare_parameter("enable_drive_cmd_bridge", False)  # 기본 OFF (안전)
        self.declare_parameter("log_throttle_sec", 1.0)
        self.declare_parameter("heartbeat_hz", 1.0)

        # ---------- Load Params ----------
        self.mqtt_host = self.get_parameter("mqtt.host").value
        self.mqtt_port = int(self.get_parameter("mqtt.port").value)
        self.mqtt_user = self.get_parameter("mqtt.username").value
        self.mqtt_pass = self.get_parameter("mqtt.password").value
        self.client_id = self.get_parameter("mqtt.client_id").value
        self.keepalive = int(self.get_parameter("mqtt.keepalive").value)
        
        
        self.robot_id = str(self.get_parameter("robot_id").value)

        self.t_sub_task_cmd = self.get_parameter("mqtt.sub.task_cmd").value
        self.t_sub_drive_cmd = self.get_parameter("mqtt.sub.drive_cmd").value
        self.t_pub_task_status = self.get_parameter("mqtt.pub.task_status").value
        self.t_pub_mission_state = self.get_parameter("mqtt.pub.mission_state").value
        self.t_pub_pose = self.get_parameter("mqtt.pub.pose").value
        self.t_pub_heartbeat = self.get_parameter("mqtt.pub.heartbeat").value

        self.ros_pub_task_cmd = self.get_parameter("ros.pub.task_cmd").value
        self.ros_pub_drive_cmd = self.get_parameter("ros.pub.drive_cmd").value
        self.ros_sub_task_status = self.get_parameter("ros.sub.task_status").value
        self.ros_sub_mission_state = self.get_parameter("ros.sub.mission_state").value
        self.ros_sub_pose = self.get_parameter("ros.sub.pose").value
        self.ros_sub_dispatch_request = self.get_parameter("ros.sub.dispatch_request").value


        self.enable_drive = bool(self.get_parameter("enable_drive_cmd_bridge").value)
        self.log_thr = float(self.get_parameter("log_throttle_sec").value)
        self.hb_hz = float(self.get_parameter("heartbeat_hz").value)
        
        
        
        tmpl_pub = self.get_parameter("mqtt.pub.dispatch_request").value
        self.t_pub_dispatch_request = tmpl_pub.replace("{robotId}", self.robot_id)

        tmpl = self.get_parameter("mqtt.sub.server_cmd").value
        self.t_sub_server_cmd = tmpl.replace("{robotId}", self.robot_id)

        self.ros_pub_server_cmd = self.get_parameter("ros.pub.server_cmd").value

        
        if self.hb_hz <= 0:
            self.hb_hz = 1.0

        self._last_log_t = 0.0

        # ---------- ROS Pub/Sub ----------
        self.pub_task_cmd = self.create_publisher(TaskCmd, self.ros_pub_task_cmd, 10)
        self.pub_drive_cmd = self.create_publisher(DriveCmd, self.ros_pub_drive_cmd, 10)
        self.pub_server_cmd = self.create_publisher(ServerCmd, self.ros_pub_server_cmd, 10)


        self.sub_task_status = self.create_subscription(TaskStatus, self.ros_sub_task_status, self._cb_task_status, 10)
        self.sub_mission_state = self.create_subscription(String, self.ros_sub_mission_state, self._cb_mission_state, 10)
        self.sub_pose = self.create_subscription(PoseStamped, self.ros_sub_pose, self._cb_pose, 10)
        self.sub_dispatch_request = self.create_subscription(
          DispatchRequest,
          self.ros_sub_dispatch_request,
          self._cb_dispatch_request,
          10
        )


        # ---------- MQTT Client ----------
        self._mqtt = mqtt.Client(client_id=self.client_id, clean_session=True)
        if self.mqtt_user:
            self._mqtt.username_pw_set(self.mqtt_user, self.mqtt_pass)

        # last will (로봇 죽었을 때)
        self._mqtt.will_set(
            self.t_pub_heartbeat,
            payload=json.dumps({"alive": False, "ts": now_unix(), "client_id": self.client_id}),
            qos=1,
            retain=True,
        )

        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect
        self._mqtt.on_message = self._on_message

        self._mqtt_lock = threading.Lock()
        self._connected = False

        # mqtt loop thread
        self._thread = threading.Thread(target=self._mqtt_loop, daemon=True)
        self._thread.start()

        # heartbeat timer
        self.timer_hb = self.create_timer(1.0 / self.hb_hz, self._tick_heartbeat)

        self.get_logger().info(
            f"mqtt_ros_bridge ready. mqtt={self.mqtt_host}:{self.mqtt_port} "
            f"sub=[{self.t_sub_task_cmd}{', '+self.t_sub_drive_cmd if self.enable_drive else ''}] "
            f"pub=[{self.t_pub_task_status},{self.t_pub_mission_state},{self.t_pub_pose}] "
            f"drive_bridge={'ON' if self.enable_drive else 'OFF'}"
        )

    # ---------------- helpers ----------------
    def _throttle_log(self, msg: str):
        t = time.time()
        if t - self._last_log_t >= self.log_thr:
            self._last_log_t = t
            self.get_logger().info(msg)

    def _mqtt_publish(self, topic: str, payload: Dict[str, Any], qos: int = 0, retain: bool = False):
        s = json.dumps(payload, ensure_ascii=False)
        with self._mqtt_lock:
            if self._connected:
                self._mqtt.publish(topic, s, qos=qos, retain=retain)

    # ---------------- MQTT callbacks ----------------
    def _on_connect(self, client, userdata, flags, rc):
        self._connected = True
        self.get_logger().info(f"MQTT connected rc={rc}")
        client.subscribe(self.t_sub_task_cmd, qos=1)
        client.subscribe(self.t_sub_server_cmd, qos=1)
        if self.enable_drive:
            client.subscribe(self.t_sub_drive_cmd, qos=1)
        # online heartbeat (retain)
        self._mqtt_publish(self.t_pub_heartbeat, {"alive": True, "ts": now_unix(), "client_id": self.client_id}, qos=1, retain=True)

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"MQTT RX invalid JSON on {msg.topic}: {e}")
            return

        if msg.topic == self.t_sub_server_cmd:
          self._handle_server_cmd(payload)
        elif msg.topic == self.t_sub_task_cmd:
          self._handle_task_cmd(payload)
        elif self.enable_drive and msg.topic == self.t_sub_drive_cmd:
          self._handle_drive_cmd(payload)


    def _mqtt_loop(self):
        # auto reconnect loop
        while rclpy.ok():
            try:
                self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=self.keepalive)
                self._mqtt.loop_forever(retry_first_connection=True)
            except Exception as e:
                self.get_logger().error(f"MQTT loop error: {e}")
                time.sleep(1.0)

    # ---------------- MQTT -> ROS handlers ----------------
    def _handle_task_cmd(self, p: Dict[str, Any]):
        """
        p 예시:
        {
          "task_id":"T-001",
          "cmd":"GOTO",
          "target_marker":5
        }
        """
        msg = TaskCmd()

        # 방어적으로 필드 셋
        if hasattr(msg, "task_id") and "task_id" in p:
            msg.task_id = str(p["task_id"])
        if hasattr(msg, "cmd") and "cmd" in p:
            msg.cmd = str(p["cmd"])
        if hasattr(msg, "target_marker") and "target_marker" in p:
            msg.target_marker = int(p["target_marker"])

        # 혹시 다른 필드들이 있으면 확장 가능
        if hasattr(msg, "timestamp"):
            msg.timestamp = int(p.get("timestamp", now_unix()))

        self.pub_task_cmd.publish(msg)
        self._throttle_log(f"MQTT->ROS task_cmd: {p}")

    def _handle_drive_cmd(self, p: Dict[str, Any]):
        """
        p 예시:
        {"enable":true,"vx":0.1,"vy":0.0,"wz":0.2,"source":"server"}
        """
        msg = DriveCmd()
        if hasattr(msg, "enable"):
            msg.enable = bool(p.get("enable", True))
        if hasattr(msg, "vx"):
            msg.vx = float(p.get("vx", 0.0))
        if hasattr(msg, "vy"):
            msg.vy = float(p.get("vy", 0.0))
        if hasattr(msg, "wz"):
            msg.wz = float(p.get("wz", 0.0))
        if hasattr(msg, "source"):
            msg.source = str(p.get("source", "server"))
        self.pub_drive_cmd.publish(msg)
        self._throttle_log(f"MQTT->ROS drive_cmd: {p}")
        
    def _handle_server_cmd(self, p: Dict[str, Any]):
        """
        서버 -> 로봇 cmd 포맷:
        {
          "type":"1",
          "reqId":"req-002",
          "cmdId":"uuid...",
          "payload":{
            "vehicleId":1,
            "targetNodeId":6,
            "path":[1,2,3,6]
          }
        }
        """
        msg = ServerCmd()

        # header stamp (있다면)
        try:
          msg.header.stamp = self.get_clock().now().to_msg()
        except Exception:
          pass
          
        msg.type = str(p.get("type", ""))
        msg.req_id = str(p.get("reqId", ""))
        msg.cmd_id = str(p.get("cmdId", ""))

        payload = p.get("payload", {})
        if not isinstance(payload, dict):
          payload = {}
        
        msg.vehicle_id = int(payload.get("vehicleId", -1))
        msg.target_node_id = int(payload.get("targetNodeId", -1))

        path = payload.get("path", [])
        if isinstance(path, list):
          msg.path = [int(x) for x in path]
        else:
          msg.path = []
        
        self.pub_server_cmd.publish(msg)
        self.get_logger().info(
          f"MQTT->ROS server_cmd: type={msg.type} req={msg.req_id} cmd={msg.cmd_id} "
          f"vehicle={msg.vehicle_id} target={msg.target_node_id} path={list(msg.path)}"
        )


    # ---------------- ROS -> MQTT callbacks ----------------
    def _cb_task_status(self, msg: TaskStatus):
        # TaskStatus를 JSON으로
        p = {"ts": now_unix()}
        for k in ["task_id", "state", "progress", "detail"]:
            if hasattr(msg, k):
                p[k] = getattr(msg, k)
        self._mqtt_publish(self.t_pub_task_status, p, qos=0, retain=False)

    def _cb_mission_state(self, msg: String):
        self._mqtt_publish(self.t_pub_mission_state, {"state": msg.data, "ts": now_unix()}, qos=0, retain=False)

    def _cb_pose(self, msg: PoseStamped):
        pos = msg.pose.position
        # yaw는 필요하면 quat->yaw 변환도 가능
        self._mqtt_publish(
            self.t_pub_pose,
            {"x": float(pos.x), "y": float(pos.y), "z": float(pos.z), "ts": now_unix()},
            qos=0,
            retain=False
        )
    def _cb_dispatch_request(self, msg: DispatchRequest):
      payload = {
          "reqId": msg.req_id,
          "plate": msg.plate,
          "ts": msg.ts,
      }
      self._mqtt_publish(self.t_pub_dispatch_request, payload, qos=1, retain=False)
      self.get_logger().info(f"ROS->MQTT dispatch_request -> {self.t_pub_dispatch_request}: {payload}")



    def _tick_heartbeat(self):
        # 연결돼있으면 주기적으로 alive
        if self._connected:
            self._mqtt_publish(self.t_pub_heartbeat, {"alive": True, "ts": now_unix(), "client_id": self.client_id}, qos=0, retain=False)


def main():
    rclpy.init()
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

