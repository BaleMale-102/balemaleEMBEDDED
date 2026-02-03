#!/usr/bin/env python3
"""
tracker_node.py - 마커 추적 노드

기능:
- 목표 마커 필터링
- Kalman Filter로 위치 스무딩
- 마커 사라짐 시 예측 모드
- TrackedMarker 메시지 퍼블리시

토픽:
  Subscribe:
    /perception/markers: robot_interfaces/MarkerArray
    /navigation/target_marker: std_msgs/Int32
  Publish:
    /perception/tracked_marker: robot_interfaces/TrackedMarker
"""

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Vector3

from .kalman_filter import KalmanFilter2D, SimpleKalman1D


class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')

        # Parameters
        self.declare_parameter('process_noise', 0.01)
        self.declare_parameter('measurement_noise', 0.05)
        self.declare_parameter('prediction_timeout', 2.0)
        self.declare_parameter('min_prediction_confidence', 0.3)
        self.declare_parameter('camera_offset_x', 0.05)  # 카메라→base_link 오프셋 (m)

        # Load parameters
        process_noise = self.get_parameter('process_noise').value
        measurement_noise = self.get_parameter('measurement_noise').value
        self.prediction_timeout = self.get_parameter('prediction_timeout').value
        self.min_confidence = self.get_parameter('min_prediction_confidence').value
        self.camera_offset_x = self.get_parameter('camera_offset_x').value  # 전방 오프셋

        # Kalman filters
        self.kf_position = KalmanFilter2D(
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )
        self.kf_angle = SimpleKalman1D(q=0.01, r=0.05)
        self.kf_yaw = SimpleKalman1D(q=0.005, r=0.02)

        # State
        self._target_marker_id = -1
        self._last_detection_time = 0.0
        self._tracking_start_time = 0.0
        self._is_tracking = False

        # Import interfaces
        try:
            from robot_interfaces.msg import MarkerArray, TrackedMarker
            self._has_interface = True
            self._MarkerArray = MarkerArray
            self._TrackedMarker = TrackedMarker
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False
            return

        # Publishers
        self.pub_tracked = self.create_publisher(
            self._TrackedMarker, '/perception/tracked_marker', 10
        )

        # Subscribers
        self.sub_markers = self.create_subscription(
            self._MarkerArray, '/perception/markers',
            self._markers_callback, 10
        )
        self.sub_target = self.create_subscription(
            Int32, '/mission/target_marker',
            self._target_callback, 10
        )

        # Timer for prediction publishing
        self.timer = self.create_timer(0.05, self._timer_callback)  # 20Hz

        self.get_logger().info('MarkerTrackerNode started')

    def _target_callback(self, msg: Int32):
        """목표 마커 ID 업데이트"""
        if msg.data != self._target_marker_id:
            self.get_logger().info(f'Target marker: {self._target_marker_id} -> {msg.data}')
            self._target_marker_id = msg.data

            # 새 타겟 → 추적 리셋
            self._reset_tracking()

    def _reset_tracking(self):
        """추적 상태 리셋"""
        self.kf_position = KalmanFilter2D(
            process_noise=self.get_parameter('process_noise').value,
            measurement_noise=self.get_parameter('measurement_noise').value
        )
        self.kf_angle = SimpleKalman1D(q=0.01, r=0.05)
        self.kf_yaw = SimpleKalman1D(q=0.005, r=0.02)
        self._is_tracking = False
        self._last_detection_time = 0.0
        self._tracking_start_time = 0.0

    def _markers_callback(self, msg):
        """마커 배열 수신 콜백"""
        if self._target_marker_id < 0:
            return

        current_time = time.time()

        # 목표 마커 찾기
        target_marker = None
        for marker in msg.markers:
            if marker.id == self._target_marker_id:
                target_marker = marker
                break

        if target_marker is not None:
            # 마커 검출됨 → Kalman 업데이트
            self._update_with_detection(target_marker, current_time)
        # 검출 안됨 → timer_callback에서 예측 처리

    def _update_with_detection(self, marker, current_time: float):
        """검출된 마커로 업데이트"""
        # 좌표 변환: OpenCV → ROS (간단화)
        # OpenCV: X-right, Y-down, Z-forward
        # 여기서는 X, Z만 사용 (수평면)
        x = marker.pose.position.x  # 좌우 (양수=오른쪽)
        z = marker.pose.position.z  # 거리 (양수=전방)

        # Kalman 업데이트
        state = self.kf_position.update(x, z, current_time)
        angle = self.kf_angle.update(marker.angle)

        # 추적 시작 시간 기록
        if not self._is_tracking:
            self._tracking_start_time = current_time
            self._is_tracking = True

        self._last_detection_time = current_time

        # TrackedMarker 퍼블리시
        self._publish_tracked(
            state=state,
            angle=angle,
            is_detected=True,
            confidence=marker.confidence,
            current_time=current_time
        )

    def _timer_callback(self):
        """주기적 예측 및 퍼블리시"""
        if not self._is_tracking or self._target_marker_id < 0:
            return

        current_time = time.time()
        time_since_detection = current_time - self._last_detection_time

        # 최근 검출이면 스킵 (markers_callback에서 처리됨)
        if time_since_detection < 0.1:
            return

        # 타임아웃 체크
        if time_since_detection > self.prediction_timeout:
            # 예측 만료
            self._publish_lost()
            return

        # 예측 모드
        state = self.kf_position.predict(0.05)  # 50ms 예측
        angle = self.kf_angle.predict()

        # 신뢰도 감소
        decay = 1.0 - (time_since_detection / self.prediction_timeout)
        confidence = max(0.0, state.confidence * decay)

        if confidence < self.min_confidence:
            self._publish_lost()
            return

        self._publish_tracked(
            state=state,
            angle=angle,
            is_detected=False,
            confidence=confidence,
            current_time=current_time
        )

    def _publish_tracked(
        self,
        state,
        angle: float,
        is_detected: bool,
        confidence: float,
        current_time: float
    ):
        """TrackedMarker 메시지 퍼블리시"""
        msg = self._TrackedMarker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.id = self._target_marker_id

        # 현재/예측 포즈
        msg.pose.position.x = state.x
        msg.pose.position.y = 0.0
        msg.pose.position.z = state.z
        msg.pose.orientation.w = 1.0

        # 예측 포즈 (미래)
        msg.predicted_pose.position.x = state.x + state.vx * 0.5
        msg.predicted_pose.position.y = 0.0
        msg.predicted_pose.position.z = state.z + state.vz * 0.5
        msg.predicted_pose.orientation.w = 1.0

        # 속도
        msg.velocity.x = state.vx
        msg.velocity.y = 0.0
        msg.velocity.z = state.vz

        # 거리, 각도 (base_link 기준: z에 카메라 오프셋 추가)
        z_base = state.z + self.camera_offset_x
        msg.distance = math.sqrt(state.x**2 + z_base**2)
        msg.angle = math.atan2(-state.x, z_base)  # base_link 기준 각도

        # 상태
        msg.is_detected = is_detected
        msg.prediction_confidence = confidence
        msg.tracking_duration = current_time - self._tracking_start_time
        msg.time_since_detection = current_time - self._last_detection_time

        self.pub_tracked.publish(msg)

    def _publish_lost(self):
        """마커 손실 메시지"""
        msg = self._TrackedMarker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.id = self._target_marker_id
        msg.is_detected = False
        msg.prediction_confidence = 0.0
        msg.time_since_detection = time.time() - self._last_detection_time

        self.pub_tracked.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
