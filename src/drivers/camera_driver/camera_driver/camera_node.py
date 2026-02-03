#!/usr/bin/env python3
"""
camera_node.py - USB 카메라 드라이버 노드

기능:
- v4l2로 USB 카메라 캡처
- sensor_msgs/Image 퍼블리시
- 시뮬레이션 모드 지원 (테스트용)

토픽:
  Publish:
    ~image_raw: sensor_msgs/Image
    ~camera_info: sensor_msgs/CameraInfo (캘리브레이션 있을 때)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('simulate', False)
        self.declare_parameter('calibration_file', '')

        # Load parameters
        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_name = self.get_parameter('camera_name').value
        self.simulate = self.get_parameter('simulate').value
        self.calib_file = self.get_parameter('calibration_file').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera capture
        self.cap = None
        if not self.simulate:
            self._init_camera()
        else:
            self.get_logger().info(f'Simulation mode enabled')

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.pub_image = self.create_publisher(
            Image,
            f'/{self.camera_name}/image_raw',
            sensor_qos
        )
        self.pub_info = self.create_publisher(
            CameraInfo,
            f'/{self.camera_name}/camera_info',
            sensor_qos
        )

        # Camera info (load from file or use defaults)
        self.camera_info = self._load_camera_info()

        # Timer for capture
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.frame_count = 0
        self.get_logger().info(
            f'CameraNode started: {self.device} @ {self.width}x{self.height} {self.fps}fps'
        )

    def _init_camera(self):
        """Initialize camera capture"""
        try:
            # Try V4L2 backend first
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

            if not self.cap.isOpened():
                self.get_logger().warn(f'V4L2 failed, trying default backend')
                self.cap = cv2.VideoCapture(self.device)

            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera: {self.device}')
                self.simulate = True
                return

            # Set properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency

            # Verify
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(
                f'Camera opened: {actual_w}x{actual_h} @ {actual_fps:.1f}fps'
            )

        except Exception as e:
            self.get_logger().error(f'Camera init error: {e}')
            self.simulate = True

    def _load_camera_info(self) -> CameraInfo:
        """Load camera info from calibration file or use defaults"""
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height

        # Default intrinsics (approximate)
        fx = self.width * 1.0  # Approximate focal length
        fy = self.width * 1.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # TODO: Load from YAML file if provided
        if self.calib_file:
            self.get_logger().info(f'Loading calibration from: {self.calib_file}')
            # Load calibration here

        return info

    def _timer_callback(self):
        """Capture and publish image"""
        timestamp = self.get_clock().now().to_msg()

        if self.simulate:
            # Generate test pattern
            frame = self._generate_test_pattern()
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to read frame', throttle_duration_sec=5.0)
                return

        # Convert to ROS message
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = self.frame_id

            # Publish image
            self.pub_image.publish(msg)

            # Publish camera info
            self.camera_info.header.stamp = timestamp
            self.pub_info.publish(self.camera_info)

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

    def _generate_test_pattern(self) -> np.ndarray:
        """Generate test pattern for simulation mode"""
        # Create gradient image
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add some visual elements
        cv2.rectangle(frame, (50, 50), (self.width - 50, self.height - 50),
                      (100, 100, 100), 2)
        cv2.putText(frame, f'Sim Frame: {self.frame_count}',
                    (60, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        cv2.putText(frame, f'{self.camera_name}',
                    (60, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        # Add moving element
        x = int((self.frame_count * 3) % (self.width - 100) + 50)
        y = int(self.height / 2 + 50 * np.sin(self.frame_count * 0.1))
        cv2.circle(frame, (x, y), 20, (0, 255, 0), -1)

        return frame

    def destroy_node(self):
        """Cleanup"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
