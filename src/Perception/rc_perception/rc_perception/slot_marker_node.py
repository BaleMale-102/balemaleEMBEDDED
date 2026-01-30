#!/usr/bin/env python3
import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped

import cv2
import numpy as np
from cv_bridge import CvBridge

try:
    from tf2_ros import TransformBroadcaster
except Exception:
    TransformBroadcaster = None


def poly_area(c4: np.ndarray) -> float:
    # c4 shape: (4,2)
    x = c4[:, 0]
    y = c4[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


class SlotMarkerNode(Node):
    """
    side_cam 전용 slot marker pose 노드.
    - ArUco detect -> slot id range만 사용 -> pose 추정 -> /perception/slot_marker_pose publish
    - Pose frame: camera_frame
      position: x forward, y left, z up (OpenCV tvec: x right, y down, z forward 변환)
    """

    def __init__(self):
        super().__init__('slot_marker_node')

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._img_stamp = None
        self._last_log_t = 0.0

        # ===== Params =====
        self.declare_parameter('image_topic', '/cam_side/image_raw')
        self.declare_parameter('camera_info_topic', '/cam_side/camera_info')
        self.declare_parameter('pose_topic', '/perception/slot_marker_pose')

        self.declare_parameter('camera_frame', 'camera_side')

        self.declare_parameter('show_debug', False)
        self.declare_parameter('log_throttle_sec', 1.0)

        # ArUco
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('id_offset', 0)

        # Slot id filter
        self.declare_parameter('slot_id_min', 16)
        self.declare_parameter('slot_id_max', 99)

        # Pose estimation needs
        self.declare_parameter('marker_size_m', 0.02)   # ✅ 너 말한 2cm 기본값
        self.declare_parameter('min_marker_area_px', 200)

        # Best selection
        self.declare_parameter('publish_best_only', True)

        # Optional TF publish (원하면 켜)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('tf_parent_frame', 'camera_side')
        self.declare_parameter('tf_child_frame', 'slot_marker')

        # ===== Pub/Sub =====
        pose_topic = str(self.get_parameter('pose_topic').value)
        self.pub_pose = self.create_publisher(PoseStamped, pose_topic, 10)

        self.sub_img = self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self.cb_img, qos_profile_sensor_data
        )
        self.sub_info = self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), self.cb_info, 10
        )

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz

        # camera intrinsics
        self.have_cam = False
        self.K = None
        self.D = None

        # aruco detector cache
        self._dict_name_cached = None
        self._dictionary_cached = None
        self._params_cached = None
        self._detector_cached = None

        # TF broadcaster
        self.tf_br = TransformBroadcaster(self) if (TransformBroadcaster is not None) else None

        self.get_logger().info(
            "slot_marker_node ready.\n"
            f" sub_img={self.get_parameter('image_topic').value}\n"
            f" sub_info={self.get_parameter('camera_info_topic').value}\n"
            f" pub_pose={pose_topic}\n"
            f" camera_frame={self.get_parameter('camera_frame').value}\n"
            f" slot_id=[{self.get_parameter('slot_id_min').value}..{self.get_parameter('slot_id_max').value}]"
        )

    def cb_img(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return
        with self._lock:
            self._frame = frame
            self._img_stamp = msg.header.stamp

    def cb_info(self, msg: CameraInfo):
        # K가 0이면 아직 준비 안 된 케이스
        if len(msg.k) != 9 or msg.k[0] == 0.0:
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        # distortion: 길이 케이스 다양 → 없으면 zeros
        if len(msg.d) > 0:
            self.D = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        else:
            self.D = np.zeros((1, 5), dtype=np.float64)

        self.have_cam = True

    def _throttle_log(self, s: str):
        now = time.time()
        thr = float(self.get_parameter('log_throttle_sec').value)
        thr = thr if thr > 0 else 1.0
        if (now - self._last_log_t) >= thr:
            self._last_log_t = now
            self.get_logger().info(s)

    def _build_detector_if_needed(self):
        if not hasattr(cv2, "aruco"):
            return

        aruco = cv2.aruco
        name = str(self.get_parameter('aruco_dict').value)

        if self._detector_cached is not None and name == self._dict_name_cached:
            return

        if not hasattr(aruco, name):
            self.get_logger().warn(f"Unknown aruco_dict={name}, fallback DICT_4X4_50")
            name = "DICT_4X4_50"

        dictionary = aruco.getPredefinedDictionary(getattr(aruco, name))
        params = aruco.DetectorParameters() if hasattr(aruco, "DetectorParameters") else aruco.DetectorParameters_create()
        if hasattr(params, "cornerRefinementMethod"):
            params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        detector = aruco.ArucoDetector(dictionary, params) if hasattr(aruco, "ArucoDetector") else None

        self._dict_name_cached = name
        self._dictionary_cached = dictionary
        self._params_cached = params
        self._detector_cached = detector

        self.get_logger().info(f"ArUco detector configured: {name}")

    def _select_best(self, kept_corners, tvecs) -> int:
        """
        best 선택: 가까운 z가 우선이지만,
        너무 작은 마커(노이즈)로 z가 튀는 케이스 방지하려고
        score = z + 0.15 / (area_norm + eps) 형태로 혼합.
        """
        if len(tvecs) <= 1:
            return 0

        areas = np.array([poly_area(c) for c in kept_corners], dtype=np.float64)
        amax = float(np.max(areas)) if float(np.max(areas)) > 1e-6 else 1.0
        an = areas / amax

        best_i = 0
        best_score = 1e9
        for i in range(len(tvecs)):
            z = float(tvecs[i][0][2])  # forward in OpenCV
            score = z + 0.15 / (float(an[i]) + 1e-3)
            if score < best_score:
                best_score = score
                best_i = i
        return best_i

    def _publish_tf(self, stamp, parent_frame: str, child_frame: str, x, y, z):
        if self.tf_br is None:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = 1.0
        self.tf_br.sendTransform(t)

    def on_timer(self):
        if not hasattr(cv2, "aruco"):
            return

        if not self.have_cam:
            # rclpy warn_throttle는 버전에 따라 없을 수 있어 직접 구현
            self._throttle_log("Waiting camera_info...")
            return

        with self._lock:
            if self._frame is None:
                return
            frame = self._frame.copy()
            stamp = self._img_stamp if self._img_stamp is not None else self.get_clock().now().to_msg()

        show = bool(self.get_parameter('show_debug').value)

        self._build_detector_if_needed()
        aruco = cv2.aruco
        dictionary = self._dictionary_cached
        params = self._params_cached
        detector = self._detector_cached

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if detector is not None:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)

        if ids is None or len(ids) == 0:
            if show:
                cv2.imshow("slot_marker_view", gray)
                cv2.waitKey(1)
            return

        id_offset = int(self.get_parameter('id_offset').value)
        slot_min = int(self.get_parameter('slot_id_min').value)
        slot_max = int(self.get_parameter('slot_id_max').value)
        min_area = float(self.get_parameter('min_marker_area_px').value)

        kept_ids = []
        kept_corners = []
        for c, mid in zip(corners, ids.flatten().tolist()):
            logical_id = int(mid) + id_offset
            if logical_id < slot_min or logical_id > slot_max:
                continue
            c4 = np.array(c, dtype=np.float32).reshape(4, 2)
            area = poly_area(c4)
            if area < min_area:
                continue
            kept_ids.append(logical_id)
            kept_corners.append(c4)

        if len(kept_ids) == 0:
            if show:
                cv2.imshow("slot_marker_view", gray)
                cv2.waitKey(1)
            return

        marker_size = float(self.get_parameter('marker_size_m').value)
        marker_size = marker_size if marker_size > 0 else 0.02

        # OpenCV API expects corners list with shape (1,4,2) per marker
        ocv_corners = [c.reshape(1, 4, 2) for c in kept_corners]
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(ocv_corners, marker_size, self.K, self.D)
        if tvecs is None or len(tvecs) == 0:
            return

        best_i = 0
        if bool(self.get_parameter('publish_best_only').value):
            best_i = self._select_best(kept_corners, tvecs)

        logical_id = kept_ids[best_i]
        t = tvecs[best_i][0]  # (x right, y down, z forward)

        # PoseStamped in camera_frame: x forward, y left, z up
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = str(self.get_parameter('camera_frame').value)

        out.pose.position.x = float(t[2])    # forward
        out.pose.position.y = float(-t[0])   # left
        out.pose.position.z = float(-t[1])   # up
        out.pose.orientation.w = 1.0

        self.pub_pose.publish(out)

        # Optional TF
        if bool(self.get_parameter('publish_tf').value):
            parent = str(self.get_parameter('tf_parent_frame').value)
            child = str(self.get_parameter('tf_child_frame').value)
            # TF도 camera 기준으로 내보내려면 parent=camera_frame로 맞춰라
            self._publish_tf(stamp, parent, child, out.pose.position.x, out.pose.position.y, out.pose.position.z)

        self._throttle_log(
            f"slot_marker_pose id={logical_id} frame={out.header.frame_id} "
            f"x={out.pose.position.x:.3f} y={out.pose.position.y:.3f} z={out.pose.position.z:.3f}"
        )

        if show:
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.aruco.drawDetectedMarkers(
                vis,
                [kept_corners[best_i].reshape(1, 4, 2)],
                np.array([[logical_id - id_offset]], dtype=np.int32)  # 화면표시는 원 aruco id로
            )
            cv2.imshow("slot_marker_view", vis)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = SlotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
