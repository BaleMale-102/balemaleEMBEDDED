#!/usr/bin/env python3
"""
anomaly_detector_node.py - 이상탐지 ROS2 노드 (소켓 클라이언트)

AI 서버(conda 환경)와 소켓으로 통신하여 추론 수행

토픽:
  Subscribe:
    /cam_front/image_raw: sensor_msgs/Image
  Publish:
    /perception/anomaly/detections: robot_interfaces/DetectionArray
    /perception/anomaly/debug_image: sensor_msgs/Image (디버그용)

실행 전 AI 서버 먼저 시작 필요:
    conda activate anpr_310
    cd /home/a102/balemaleAI
    python servers/anomaly_server.py
"""

import socket
import struct
import pickle

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robot_interfaces.msg import Detection, DetectionArray


# 소켓 통신 함수
def send_data(sock: socket.socket, data: dict):
    """데이터를 소켓으로 전송"""
    payload = pickle.dumps(data)
    header = struct.pack('>I', len(payload))
    sock.sendall(header + payload)


def recv_data(sock: socket.socket) -> dict:
    """소켓에서 데이터 수신"""
    header = _recv_exact(sock, 4)
    if not header:
        return None
    length = struct.unpack('>I', header)[0]
    payload = _recv_exact(sock, length)
    if not payload:
        return None
    return pickle.loads(payload)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    """정확히 n바이트 수신"""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


class AnomalyDetectorNode(Node):
    CLASS_NAMES = {
        2: 'person',
        3: 'box',
        4: 'cone',
    }

    def __init__(self):
        super().__init__('anomaly_detector')

        # Parameters
        self.declare_parameter('image_topic', '/cam_front/image_raw')
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 9001)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_debug_window', False)

        # Load parameters
        image_topic = self.get_parameter('image_topic').value
        self.server_host = self.get_parameter('server_host').value
        self.server_port = self.get_parameter('server_port').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.show_debug_window = self.get_parameter('show_debug_window').value

        # CV Bridge
        self.bridge = CvBridge()

        # Socket connection
        self.socket = None
        self._connect_to_server()

        # Publishers
        self.pub_detections = self.create_publisher(
            DetectionArray, '/perception/anomaly/detections', 10
        )

        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/perception/anomaly/debug_image', 10
            )

        # Subscriber
        self.sub_image = self.create_subscription(
            Image, image_topic, self._image_callback,
            qos_profile_sensor_data
        )

        # 첫 검출 로깅용 플래그
        self.first_detection_logged = False

        self.get_logger().info(
            f'AnomalyDetectorNode started. Server: {self.server_host}:{self.server_port}'
        )

    def _connect_to_server(self):
        """AI 서버에 연결"""
        try:
            if self.socket:
                self.socket.close()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_host, self.server_port))
            self.get_logger().info(f'Connected to AI server')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to AI server: {e}')
            self.socket = None

    def _image_callback(self, msg: Image):
        """이미지 수신 콜백"""
        if self.socket is None:
            self._connect_to_server()
            if self.socket is None:
                return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        try:
            # 서버에 이미지 전송
            send_data(self.socket, {'image': frame})

            # 결과 수신
            result = recv_data(self.socket)
            if result is None:
                self.get_logger().warn('No response from server, reconnecting...')
                self._connect_to_server()
                return

            detections = result.get('detections', [])

            # DetectionArray 메시지 생성
            det_array_msg = DetectionArray()
            det_array_msg.header = msg.header
            det_array_msg.num_plates = 0
            det_array_msg.num_obstacles = len(detections)

            for det in detections:
                detection_msg = Detection()
                detection_msg.class_id = det['class']
                detection_msg.class_name = self.CLASS_NAMES.get(det['class'], 'unknown')
                detection_msg.confidence = float(det['confidence'])
                detection_msg.x1 = int(det['box'][0])
                detection_msg.y1 = int(det['box'][1])
                detection_msg.x2 = int(det['box'][2])
                detection_msg.y2 = int(det['box'][3])
                detection_msg.distance = float(det.get('distance', 0.0))
                det_array_msg.detections.append(detection_msg)

            self.pub_detections.publish(det_array_msg)

            # 로깅 (첫 검출에만)
            if detections and not self.first_detection_logged:
                summary = ', '.join([
                    f"{self.CLASS_NAMES.get(d['class'], '?')}({d.get('distance', 0):.1f}cm)"
                    for d in detections
                ])
                self.get_logger().info(f'Anomaly: {summary}')
                self.first_detection_logged = True

            # 디버그 이미지
            if self.publish_debug or self.show_debug_window:
                debug_image = self._draw_detections(frame, detections)

                if self.publish_debug:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                    debug_msg.header = msg.header
                    self.pub_debug.publish(debug_msg)

                if self.show_debug_window:
                    cv2.imshow('Anomaly Detector', debug_image)
                    cv2.waitKey(1)

        except (BrokenPipeError, ConnectionResetError) as e:
            self.get_logger().warn(f'Connection lost: {e}, reconnecting...')
            self._connect_to_server()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def _draw_detections(self, frame, detections):
        """검출 결과를 이미지에 그리기"""
        result = frame.copy()
        colors = {
            2: (0, 0, 255),    # Person - Red
            3: (255, 255, 0),  # Box - Cyan
            4: (0, 165, 255),  # Cone - Orange
        }

        for det in detections:
            x1, y1, x2, y2 = map(int, det['box'])
            cls = det['class']
            conf = det['confidence']
            dist = det.get('distance', 0.0)
            color = colors.get(cls, (255, 255, 255))
            label = self.CLASS_NAMES.get(cls, f'Class{cls}')

            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                result, f'{label} {dist:.1f}cm',
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 2
            )

        return result

    def destroy_node(self):
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
