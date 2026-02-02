#!/usr/bin/env python3
"""
detector_node.py - 라인 검출 노드

기능:
- 하단 카메라 이미지에서 라인 검출
- Kalman Filter로 스무딩
- LaneStatus 메시지 퍼블리시

토픽:
  Subscribe:
    ~image_raw: sensor_msgs/Image
  Publish:
    /perception/lane_status: robot_interfaces/LaneStatus
    /perception/debug/lane_image: sensor_msgs/Image (선택)
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .lane_detector import LaneDetector


class SimpleKalman1D:
    """간단한 1D Kalman Filter"""

    def __init__(self, q=0.01, r=0.05):
        self.q = q
        self.r = r
        self.x = 0.0
        self.p = 1.0

    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x

    def reset(self, value=0.0):
        self.x = value
        self.p = 1.0


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_debug_window', False)
        self.declare_parameter('roi_top_ratio', 0.5)
        self.declare_parameter('roi_bottom_ratio', 1.0)
        self.declare_parameter('sobel_threshold', 50)
        self.declare_parameter('min_line_pixels', 100)
        self.declare_parameter('line_color', 'black')
        self.declare_parameter('kalman_q', 0.01)
        self.declare_parameter('kalman_r', 0.05)

        # Load parameters
        image_topic = self.get_parameter('image_topic').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.show_debug_window = self.get_parameter('show_debug_window').value

        # Lane detector
        self.detector = LaneDetector(
            roi_top_ratio=self.get_parameter('roi_top_ratio').value,
            roi_bottom_ratio=self.get_parameter('roi_bottom_ratio').value,
            sobel_threshold=self.get_parameter('sobel_threshold').value,
            min_line_pixels=self.get_parameter('min_line_pixels').value,
            line_color=self.get_parameter('line_color').value
        )

        # Kalman filters for smoothing
        kalman_q = self.get_parameter('kalman_q').value
        kalman_r = self.get_parameter('kalman_r').value
        self.kf_offset = SimpleKalman1D(kalman_q, kalman_r)
        self.kf_angle = SimpleKalman1D(kalman_q, kalman_r)

        # CV Bridge
        self.bridge = CvBridge()

        # Import interfaces
        try:
            from robot_interfaces.msg import LaneStatus
            self._has_interface = True
            self._LaneStatus = LaneStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        # Publishers
        if self._has_interface:
            self.pub_lane = self.create_publisher(
                self._LaneStatus, '/perception/lane_status', 10
            )

        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/perception/debug/lane_image', 10
            )

        # Subscribers
        self.sub_image = self.create_subscription(
            Image, image_topic, self._image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'LaneDetectorNode started: {image_topic}')

    def _image_callback(self, msg: Image):
        """이미지 수신 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # 라인 검출
        result = self.detector.detect(cv_image, generate_debug=self.publish_debug)

        # Kalman 필터 적용
        if result.valid:
            filtered_offset = self.kf_offset.update(result.offset_normalized)
            filtered_angle = self.kf_angle.update(result.angle)
        else:
            filtered_offset = self.kf_offset.x
            filtered_angle = self.kf_angle.x

        # LaneStatus 퍼블리시
        if self._has_interface:
            lane_msg = self._LaneStatus()
            lane_msg.header.stamp = msg.header.stamp
            lane_msg.header.frame_id = 'camera_front_link'

            lane_msg.valid = result.valid
            lane_msg.offset = float(result.offset)
            lane_msg.offset_normalized = float(filtered_offset)
            lane_msg.angle = float(filtered_angle)
            lane_msg.curvature = float(result.curvature)
            lane_msg.confidence = float(result.confidence)

            self.pub_lane.publish(lane_msg)

        # 디버그 이미지
        if self.publish_debug and result.debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(result.debug_image, 'bgr8')
                debug_msg.header = msg.header
                self.pub_debug.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Debug image error: {e}')

        # 디버그 창 표시
        if self.show_debug_window and result.debug_image is not None:
            cv2.imshow('Lane Detector', result.debug_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

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
