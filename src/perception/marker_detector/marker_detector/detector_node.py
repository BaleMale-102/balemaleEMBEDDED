#!/usr/bin/env python3
"""
detector_node.py - ArUco 마커 검출 노드

기능:
- 카메라 이미지에서 ArUco 마커 검출
- MarkerArray 메시지 퍼블리시
- 디버그 이미지 퍼블리시 (선택)

토픽:
  Subscribe:
    ~image_raw: sensor_msgs/Image
    ~camera_info: sensor_msgs/CameraInfo
  Publish:
    /perception/markers: robot_interfaces/MarkerArray
    /perception/debug/marker_image: sensor_msgs/Image (선택)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import math

from .aruco_detector import ArucoDetector


class MarkerDetectorNode(Node):
    def __init__(self):
        super().__init__('marker_detector_node')

        # Parameters
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_debug_window', False)
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/front/camera_info')
        self.declare_parameter('frame_id', 'camera_front_link')

        # Load parameters
        dictionary = self.get_parameter('dictionary').value
        marker_size = self.get_parameter('marker_size').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.show_debug_window = self.get_parameter('show_debug_window').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Detector
        self.detector = ArucoDetector(
            dictionary=dictionary,
            marker_size=marker_size
        )

        # CV Bridge
        self.bridge = CvBridge()

        # Camera info received flag
        self._camera_info_received = False

        # Publishers
        try:
            from robot_interfaces.msg import Marker, MarkerArray
            self.pub_markers = self.create_publisher(
                MarkerArray, '/perception/markers', 10
            )
            self._has_interface = True
            self._Marker = Marker
            self._MarkerArray = MarkerArray
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/perception/debug/marker_image', 10
            )

        # Subscribers
        self.sub_image = self.create_subscription(
            Image, image_topic, self._image_callback,
            qos_profile_sensor_data
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'MarkerDetectorNode started: {dictionary}, marker_size={marker_size}m'
        )

    def _camera_info_callback(self, msg: CameraInfo):
        """카메라 정보 수신 콜백"""
        if self._camera_info_received:
            return

        # 카메라 매트릭스 추출
        camera_matrix = np.array(msg.k).reshape(3, 3)
        dist_coeffs = np.array(msg.d)

        if dist_coeffs.size < 5:
            dist_coeffs = np.zeros(5)

        self.detector.set_camera_params(camera_matrix, dist_coeffs)
        self._camera_info_received = True

        self.get_logger().info(
            f'Camera info received: {msg.width}x{msg.height}'
        )

    def _image_callback(self, msg: Image):
        """이미지 수신 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # 마커 검출
        markers = self.detector.detect(cv_image)

        # 메시지 퍼블리시
        if self._has_interface:
            self._publish_markers(markers, msg.header.stamp)

        # 디버그 이미지
        if self.publish_debug or self.show_debug_window:
            debug_image = self.detector.draw_markers(cv_image, markers)

            if self.publish_debug:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                    debug_msg.header = msg.header
                    self.pub_debug.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Debug image error: {e}')

            # 디버그 창 표시 (노드 이름으로 구분)
            if self.show_debug_window:
                window_name = f'Marker Detector - {self.get_name()}'
                cv2.imshow(window_name, debug_image)
                cv2.waitKey(1)

    def _publish_markers(self, markers, stamp):
        """MarkerArray 메시지 퍼블리시"""
        msg = self._MarkerArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        for m in markers:
            marker_msg = self._Marker()
            marker_msg.id = m.id

            # 포즈 (카메라 좌표계)
            # OpenCV: X-right, Y-down, Z-forward
            # 그대로 유지 (ROS 변환은 tracker에서)
            marker_msg.pose.position.x = float(m.tvec[0])
            marker_msg.pose.position.y = float(m.tvec[1])
            marker_msg.pose.position.z = float(m.tvec[2])

            # 쿼터니언: 2D yaw 기반 (카메라 pitch 영향 최소화)
            # m.yaw는 이제 2D 이미지 기반 회전 (마커 코너에서 계산)
            # Z축 회전만 있는 단순 quaternion으로 변환
            yaw = m.yaw
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = math.sin(yaw / 2.0)
            marker_msg.pose.orientation.w = math.cos(yaw / 2.0)

            marker_msg.distance = float(m.distance)
            marker_msg.angle = float(m.angle)
            marker_msg.confidence = float(m.confidence)

            msg.markers.append(marker_msg)

        self.pub_markers.publish(msg)

    def _rvec_to_quaternion(self, rvec: np.ndarray) -> tuple:
        """회전 벡터를 쿼터니언으로 변환"""
        import cv2
        R, _ = cv2.Rodrigues(rvec)

        # 회전 행렬에서 쿼터니언 계산
        trace = R[0, 0] + R[1, 1] + R[2, 2]

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorNode()

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
