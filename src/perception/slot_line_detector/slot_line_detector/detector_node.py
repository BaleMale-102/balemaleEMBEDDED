#!/usr/bin/env python3
"""
detector_node.py - Slot Line Detector Node

Detects yellow tape rectangles (parking slot boundaries) from side camera.
Publishes SlotLineStatus with rectangle position and angle.

Subscribes:
    /cam_side/image_raw: Image

Publishes:
    /perception/slot_rect: SlotLineStatus
    /slot_line_detector/debug_image: Image (optional)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from .rectangle_detector import RectangleDetector


class SlotLineDetectorNode(Node):
    """ROS2 node for yellow rectangle detection."""

    def __init__(self):
        super().__init__('slot_line_detector')

        # Parameters
        self.declare_parameter('image_topic', '/cam_side/image_raw')
        self.declare_parameter('output_topic', '/perception/slot_rect')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_debug_window', False)

        # HSV thresholds for yellow
        self.declare_parameter('hsv_low_h', 20)
        self.declare_parameter('hsv_low_s', 100)
        self.declare_parameter('hsv_low_v', 100)
        self.declare_parameter('hsv_high_h', 35)
        self.declare_parameter('hsv_high_s', 255)
        self.declare_parameter('hsv_high_v', 255)

        # Detection parameters
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('min_aspect_ratio', 0.3)
        self.declare_parameter('max_aspect_ratio', 3.0)

        # Pixel to meter conversion (approximate, needs calibration)
        self.declare_parameter('pixel_to_meter', 0.001)  # 1 pixel = 1mm

        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.show_window = self.get_parameter('show_debug_window').value
        self.pixel_to_meter = self.get_parameter('pixel_to_meter').value

        # Build HSV thresholds
        hsv_low = (
            self.get_parameter('hsv_low_h').value,
            self.get_parameter('hsv_low_s').value,
            self.get_parameter('hsv_low_v').value,
        )
        hsv_high = (
            self.get_parameter('hsv_high_h').value,
            self.get_parameter('hsv_high_s').value,
            self.get_parameter('hsv_high_v').value,
        )

        min_area = self.get_parameter('min_area').value
        max_area = self.get_parameter('max_area').value
        min_aspect = self.get_parameter('min_aspect_ratio').value
        max_aspect = self.get_parameter('max_aspect_ratio').value

        # Initialize detector
        self.detector = RectangleDetector(
            hsv_low=hsv_low,
            hsv_high=hsv_high,
            min_area=min_area,
            max_area=max_area,
            min_aspect_ratio=min_aspect,
            max_aspect_ratio=max_aspect,
        )

        # CV Bridge
        self.bridge = CvBridge()

        # Import custom message
        try:
            from robot_interfaces.msg import SlotLineStatus
            self._has_interface = True
            self._SlotLineStatus = SlotLineStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found, using dummy output')
            self._has_interface = False

        # Publishers
        if self._has_interface:
            self.pub_status = self.create_publisher(
                self._SlotLineStatus, output_topic, 10
            )

        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/slot_line_detector/debug_image', qos_profile_sensor_data
            )

        # Subscriber
        self.sub_image = self.create_subscription(
            Image, image_topic,
            self._image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'SlotLineDetectorNode started: {image_topic} -> {output_topic}'
        )

    def _image_callback(self, msg: Image):
        """Process incoming image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Detect rectangle
        result = self.detector.detect(cv_image)

        # Publish status
        if self._has_interface:
            status_msg = self._SlotLineStatus()
            status_msg.header = msg.header
            status_msg.valid = result.valid
            status_msg.center_offset_x = result.offset_x * self.pixel_to_meter
            status_msg.center_offset_y = result.offset_y * self.pixel_to_meter
            status_msg.angle = result.angle
            status_msg.width = result.width
            status_msg.height = result.height
            status_msg.confidence = result.confidence
            self.pub_status.publish(status_msg)

        # Debug visualization
        if self.publish_debug or self.show_window:
            debug_img = self.detector.draw_debug(cv_image, result)

            if self.publish_debug:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
                debug_msg.header = msg.header
                self.pub_debug.publish(debug_msg)

            if self.show_window:
                cv2.imshow('Slot Line Detector', debug_img)
                cv2.waitKey(1)

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlotLineDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
