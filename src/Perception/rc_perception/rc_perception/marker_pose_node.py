#!/usr/bin/env python3
"""
marker_pose_node.py - ArUco marker pose estimation
발행:
  - /perception/marker_pose (PoseStamped) - 카메라 프레임 기준 상대 pose
  - /perception/marker_status (MarkerStatus) - id, quality, 상대좌표
"""
import time
import threading
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from rc_interfaces.msg import MarkerStatus

import cv2
import numpy as np
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster


def estimate_pose_single_markers(corners: List[np.ndarray],
                                  marker_length: float,
                                  camera_matrix: np.ndarray,
                                  dist_coeffs: np.ndarray):
    """OpenCV 4.7+ 호환: solvePnP 기반 pose estimation"""
    half = marker_length / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    rvecs, tvecs = [], []
    for corner in corners:
        img_points = np.array(corner, dtype=np.float64).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if success:
            rvecs.append(rvec)
            tvecs.append(tvec)
        else:
            rvecs.append(np.zeros((3, 1)))
            tvecs.append(np.zeros((3, 1)))
    return rvecs, tvecs


class MarkerPoseNode(Node):
    def __init__(self):
        super().__init__('marker_pose_node')

        if not hasattr(cv2, "aruco"):
            self.get_logger().error("cv2.aruco not available!")
            raise RuntimeError("cv2.aruco not available")

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None

        # Camera info
        self._have_caminfo = False
        self._camera_matrix = None
        self._dist_coeffs = None
        self._cam_frame = ""

        # Tracking state
        self._last_valid_t = 0.0
        self._last_id: Optional[int] = None
        self._ema_tvec = None
        self._ema_rvec = None

        # Parameters
        self.declare_parameter('image_topic', '/cam_front/image_raw')
        self.declare_parameter('camera_info_topic', '/cam_front/camera_info')
        self.declare_parameter('pose_topic', '/perception/marker_pose')
        self.declare_parameter('status_topic', '/perception/marker_status')

        self.declare_parameter('show_debug', False)
        self.declare_parameter('log_throttle_sec', 1.0)

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_length_m', 0.04)
        self.declare_parameter('id_offset', 0)

        self.declare_parameter('min_marker_area_px', 100.0)
        self.declare_parameter('quality_area_ref_px', 2500.0)
        self.declare_parameter('target_id', -1)

        self.declare_parameter('prefer_last_id', True)
        self.declare_parameter('prefer_last_id_sec', 0.35)

        self.declare_parameter('min_z_m', 0.02)
        self.declare_parameter('max_z_m', 2.0)
        self.declare_parameter('max_xy_m', 2.0)
        self.declare_parameter('max_step_m', 0.3)

        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('ema_alpha', 0.4)

        self.declare_parameter('timer_period_sec', 0.05)

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_child_prefix', 'marker_')

        # Publishers
        self.pub_pose = self.create_publisher(
            PoseStamped,
            self.get_parameter('pose_topic').value, 10)
        self.pub_status = self.create_publisher(
            MarkerStatus,
            self.get_parameter('status_topic').value, 10)

        # Subscribers
        self.sub_img = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.cb_image, qos_profile_sensor_data)
        self.sub_caminfo = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.cb_caminfo, qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self)

        # ArUco setup
        self._setup_aruco()

        self._last_log_t = 0.0
        period = float(self.get_parameter('timer_period_sec').value)
        self.timer = self.create_timer(period, self.process_frame)

        self.get_logger().info(
            f"marker_pose_node initialized\n"
            f"  image: {self.get_parameter('image_topic').value}\n"
            f"  pose: {self.get_parameter('pose_topic').value}\n"
            f"  status: {self.get_parameter('status_topic').value}\n"
            f"  marker_length: {self.get_parameter('marker_length_m').value}m"
        )

    def _setup_aruco(self):
        aruco = cv2.aruco
        dict_name = str(self.get_parameter('aruco_dict').value)

        if not hasattr(aruco, dict_name):
            dict_name = 'DICT_4X4_50'

        self._dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))

        if hasattr(aruco, "DetectorParameters"):
            self._detector_params = aruco.DetectorParameters()
        else:
            self._detector_params = aruco.DetectorParameters_create()

        if hasattr(self._detector_params, "cornerRefinementMethod"):
            self._detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        if hasattr(aruco, "ArucoDetector"):
            self._detector = aruco.ArucoDetector(self._dictionary, self._detector_params)
        else:
            self._detector = None

        self.get_logger().info(f"ArUco dictionary: {dict_name}")

    def cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._frame = frame
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    def cb_caminfo(self, msg: CameraInfo):
        if msg.k[0] == 0.0:
            return
        with self._lock:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)
            if msg.header.frame_id:
                self._cam_frame = msg.header.frame_id
            self._have_caminfo = True

    def _log_throttle(self, msg: str):
        now = time.time()
        interval = float(self.get_parameter('log_throttle_sec').value)
        if (now - self._last_log_t) >= interval:
            self._last_log_t = now
            self.get_logger().info(msg)

    @staticmethod
    def _polygon_area(pts: np.ndarray) -> float:
        x, y = pts[:, 0], pts[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    @staticmethod
    def _rvec_to_quaternion(rvec: np.ndarray) -> Tuple[float, float, float, float]:
        R, _ = cv2.Rodrigues(rvec)
        trace = R[0, 0] + R[1, 1] + R[2, 2]

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return (float(x), float(y), float(z), float(w))

    @staticmethod
    def _rvec_to_yaw(rvec: np.ndarray) -> float:
        """rvec에서 yaw (Z축 회전) 추출"""
        R, _ = cv2.Rodrigues(rvec)
        return float(np.arctan2(R[1, 0], R[0, 0]))

    def _select_best_marker(self, corners, ids):
        if ids is None or len(ids) == 0:
            return None

        target_id = int(self.get_parameter('target_id').value)
        min_area = float(self.get_parameter('min_marker_area_px').value)
        area_ref = float(self.get_parameter('quality_area_ref_px').value)
        prefer_last = bool(self.get_parameter('prefer_last_id').value)
        prefer_sec = float(self.get_parameter('prefer_last_id_sec').value)

        now = time.time()
        candidates = {}

        for corner, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            pts = np.array(corner).reshape(4, 2)
            area = self._polygon_area(pts)

            if area < min_area:
                continue

            quality = min(area / area_ref, 1.0)
            candidates[marker_id] = (pts, quality)

        if not candidates:
            return None

        if target_id >= 0 and target_id in candidates:
            pts, q = candidates[target_id]
            return (target_id, pts, q)

        if prefer_last and self._last_id in candidates:
            if (now - self._last_valid_t) < prefer_sec:
                pts, q = candidates[self._last_id]
                return (self._last_id, pts, q)

        best_id = max(candidates, key=lambda k: candidates[k][1])
        pts, q = candidates[best_id]
        return (best_id, pts, q)

    def _ema_filter(self, prev: np.ndarray, curr: np.ndarray, alpha: float) -> np.ndarray:
        return prev * (1.0 - alpha) + curr * alpha

    def _publish_invalid_status(self):
        """마커 미검출시 invalid status 발행"""
        status = MarkerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self._cam_frame if self._cam_frame else "camera_front"
        status.valid = False
        status.id = -1
        status.quality = 0.0
        status.rel_x = 0.0
        status.rel_y = 0.0
        status.rel_z = 0.0
        status.rel_yaw = 0.0
        self.pub_status.publish(status)

    def process_frame(self):
        show_debug = bool(self.get_parameter('show_debug').value)

        with self._lock:
            if self._frame is None:
                return
            image = self._frame.copy()

            if not self._have_caminfo or self._camera_matrix is None:
                if show_debug:
                    cv2.imshow("marker_pose", image)
                    cv2.waitKey(1)
                self._log_throttle("Waiting for CameraInfo...")
                self._publish_invalid_status()
                return

            camera_matrix = self._camera_matrix.copy()
            dist_coeffs = self._dist_coeffs.copy()
            cam_frame = self._cam_frame

        marker_length = float(self.get_parameter('marker_length_m').value)

        # ArUco detection
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image, self._dictionary, parameters=self._detector_params)

        selection = self._select_best_marker(corners, ids)

        if selection is None:
            if show_debug:
                cv2.imshow("marker_pose", image)
                cv2.waitKey(1)
            self._publish_invalid_status()
            return

        marker_id, corner_pts, quality = selection

        # Pose estimation
        rvecs, tvecs = estimate_pose_single_markers(
            [corner_pts], marker_length, camera_matrix, dist_coeffs)

        rvec = rvecs[0].reshape(3, 1)
        tvec = tvecs[0].reshape(3, 1)

        # Validation
        x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])
        min_z = float(self.get_parameter('min_z_m').value)
        max_z = float(self.get_parameter('max_z_m').value)
        max_xy = float(self.get_parameter('max_xy_m').value)

        if not (min_z < z < max_z):
            self._log_throttle(f"Rejected: z={z:.3f}m out of range")
            self._publish_invalid_status()
            return

        if abs(x) > max_xy or abs(y) > max_xy:
            self._log_throttle(f"Rejected: xy out of range")
            self._publish_invalid_status()
            return

        # Step jump rejection
        max_step = float(self.get_parameter('max_step_m').value)
        if self._ema_tvec is not None:
            step = float(np.linalg.norm(tvec - self._ema_tvec))
            if step > max_step:
                self._log_throttle(f"Rejected: step={step:.3f}m")
                self._publish_invalid_status()
                return

        # Smoothing
        if bool(self.get_parameter('enable_smoothing').value):
            alpha = float(self.get_parameter('ema_alpha').value)
            if self._ema_tvec is None:
                self._ema_tvec = tvec.copy()
                self._ema_rvec = rvec.copy()
            else:
                self._ema_tvec = self._ema_filter(self._ema_tvec, tvec, alpha)
                self._ema_rvec = self._ema_filter(self._ema_rvec, rvec, alpha)
            tvec_out = self._ema_tvec
            rvec_out = self._ema_rvec
        else:
            tvec_out = tvec
            rvec_out = rvec

        now_stamp = self.get_clock().now().to_msg()
        frame_id = cam_frame if cam_frame else "camera_front"

        # === Publish PoseStamped ===
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(tvec_out[0])
        pose_msg.pose.position.y = float(tvec_out[1])
        pose_msg.pose.position.z = float(tvec_out[2])

        qx, qy, qz, qw = self._rvec_to_quaternion(rvec_out)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pub_pose.publish(pose_msg)

        # === Publish MarkerStatus ===
        status_msg = MarkerStatus()
        status_msg.header.stamp = now_stamp
        status_msg.header.frame_id = frame_id
        status_msg.valid = True
        status_msg.id = marker_id + int(self.get_parameter('id_offset').value)
        status_msg.quality = float(quality)
        status_msg.rel_x = float(tvec_out[0])
        status_msg.rel_y = float(tvec_out[1])
        status_msg.rel_z = float(tvec_out[2])
        status_msg.rel_yaw = self._rvec_to_yaw(rvec_out)

        self.pub_status.publish(status_msg)

        # === TF broadcast ===
        if bool(self.get_parameter('publish_tf').value):
            prefix = str(self.get_parameter('tf_child_prefix').value)
            offset = int(self.get_parameter('id_offset').value)

            tf_msg = TransformStamped()
            tf_msg.header = pose_msg.header
            tf_msg.child_frame_id = f"{prefix}{marker_id + offset}"
            tf_msg.transform.translation.x = pose_msg.pose.position.x
            tf_msg.transform.translation.y = pose_msg.pose.position.y
            tf_msg.transform.translation.z = pose_msg.pose.position.z
            tf_msg.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        # Update state
        self._last_id = marker_id
        self._last_valid_t = time.time()

        self._log_throttle(
            f"id={marker_id} q={quality:.2f} "
            f"t=[{tvec_out[0,0]:.3f}, {tvec_out[1,0]:.3f}, {tvec_out[2,0]:.3f}]"
        )

        # Debug visualization
        if show_debug:
            image_copy = image.copy()
            cv2.aruco.drawDetectedMarkers(
                image_copy, [corner_pts.reshape(1, 4, 2)],
                np.array([[marker_id]], dtype=np.int32))
            cv2.drawFrameAxes(
                image_copy, camera_matrix, dist_coeffs,
                rvec_out, tvec_out, marker_length * 0.5)

            cv2.putText(image_copy, f"x: {tvec_out[0,0]:.3f}m", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image_copy, f"y: {tvec_out[1,0]:.3f}m", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image_copy, f"z: {tvec_out[2,0]:.3f}m", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image_copy, f"id: {marker_id}", (10, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            cv2.imshow("marker_pose", image_copy)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = MarkerPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()