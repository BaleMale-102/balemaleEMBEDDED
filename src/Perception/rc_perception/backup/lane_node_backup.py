#!/usr/bin/env python3
import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge

from rc_interfaces.msg import LaneStatus


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class LaneNode(Node):
    """
    Bottom-camera lane detector (black lane) -> /perception/lane (rc_interfaces/LaneStatus)

    ✅ 안정화 포인트(실차용)
    - lane.offset: "largest contour only" 마스크에서 median(cx)로 계산 (잡음/그림자 방지)
    - lane.angle: cv2.fitLine 방향 부호를 위쪽(-y)로 고정해 프레임별 부호 뒤집힘 방지
    - angle 샘플링: 랜덤 대신 균등 간격 샘플링(재현성/지터 감소)
    - debug: contour-mask 기준으로 표시
    """

    def __init__(self):
        super().__init__('lane_node')
        self.bridge = CvBridge()

        # Params
        self.declare_parameter('image_topic', '/cam_bottom/image_raw')
        self.declare_parameter('show_debug', False)

        # ROI params
        self.declare_parameter('crop_h_start_ratio', 0.5)   # 아래 절반 (0~1)

        # threshold / morphology
        self.declare_parameter('black_v_max', 60)           # HSV V 상한
        self.declare_parameter('erode_iter', 2)
        self.declare_parameter('dilate_iter', 2)
        self.declare_parameter('kernel', 5)

        # quality / gating
        self.declare_parameter('min_contour_area', 200)
        self.declare_parameter('log_throttle_sec', 1.0)

        # angle estimation params
        self.declare_parameter('angle_max_deg', 35.0)       # angle clamp (deg)
        self.declare_parameter('angle_sample_max', 6000)    # max points for fitLine

        # header.frame_id (control_stack에서 pose랑 섞일 수 있으니 명시)
        self.declare_parameter('frame_id', 'camera_bottom')

        self._last_log_t = 0.0
        self._lock = threading.Lock()
        self._frame = None

        # Pub/Sub
        self.pub_lane = self.create_publisher(LaneStatus, '/perception/lane', 10)

        image_topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(Image, image_topic, self.cb_img, qos_profile_sensor_data)

        self.timer = self.create_timer(0.03, self.on_timer)  # ~33Hz
        self.get_logger().info(f"lane_node ready. sub={image_topic} pub=/perception/lane")

    def cb_img(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return
        with self._lock:
            self._frame = frame

    def _throttle_log(self, s: str):
        now = time.time()
        thr = float(self.get_parameter('log_throttle_sec').value)
        if thr <= 0:
            thr = 1.0
        if (now - self._last_log_t) >= thr:
            self._last_log_t = now
            # self.get_logger().info(s)

    def _estimate_angle_rad(self, mask_c: np.ndarray) -> float:
        """
        mask_c (largest contour mask)에서 lane 방향각을 추정.
        - 0 rad: 직진(세로/위쪽)
        - +: 오른쪽으로 기울어짐
        """
        ys, xs = np.where(mask_c > 0)
        if xs.size < 80:
            return 0.0

        max_n = int(self.get_parameter('angle_sample_max').value)
        if xs.size > max_n:
            # ✅ 랜덤 대신 균등 샘플링(프레임 지터 감소)
            step = max(1, xs.size // max_n)
            xs = xs[::step][:max_n]
            ys = ys[::step][:max_n]

        pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)

        vx, vy, _, _ = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(vx)
        vy = float(vy)

        # ✅ 방향 고정: 항상 위쪽(-y)으로 향하게
        if vy > 0.0:
            vx = -vx
            vy = -vy

        # 세로(위쪽) 기준 각도
        angle = math.atan2(vx, -vy)

        ang_max = math.radians(float(self.get_parameter('angle_max_deg').value))
        return float(clamp(angle, -ang_max, +ang_max))

    def on_timer(self):
        with self._lock:
            if not isinstance(self._frame, np.ndarray):
                return
            frame = self._frame.copy()

        h, w, _ = frame.shape

        # ROI: 아래쪽 영역
        ratio = float(self.get_parameter('crop_h_start_ratio').value)
        ratio = clamp(ratio, 0.0, 0.95)
        crop_h_start = int(h * ratio)
        if crop_h_start >= h - 2:
            crop_h_start = max(0, h - 2)

        crop = frame[crop_h_start:h, 0:w].copy()
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        # 검정(길) 마스크
        v_max = int(self.get_parameter('black_v_max').value)
        v_max = int(clamp(v_max, 0, 255))
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # morphology
        k = int(self.get_parameter('kernel').value)
        k = int(clamp(k, 1, 31))
        kernel = np.ones((k, k), np.uint8)

        e_iter = int(self.get_parameter('erode_iter').value)
        d_iter = int(self.get_parameter('dilate_iter').value)
        if e_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=e_iter)
        if d_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=d_iter)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lane = LaneStatus()
        lane.header.stamp = self.get_clock().now().to_msg()
        lane.header.frame_id = str(self.get_parameter('frame_id').value)

        # default: no lane
        lane.in_lane = False
        lane.offset_px = 0.0
        lane.offset_norm = 0.0
        lane.quality = 0.0
        lane.angle = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(c))
            min_area = float(self.get_parameter('min_contour_area').value)

            if area >= min_area:
                # ✅ largest contour only mask (잡음/그림자 방지)
                mask_c = np.zeros_like(mask)
                cv2.drawContours(mask_c, [c], -1, 255, thickness=-1)

                # ✅ center: contour mask에서 median
                xs_c = np.where(mask_c > 0)[1]
                if xs_c.size > 0:
                    cx = int(np.median(xs_c))
                else:
                    M = cv2.moments(c)
                    if M['m00'] > 1e-6:
                        cx = int(M['m10'] / M['m00'])
                    else:
                        self.pub_lane.publish(lane)
                        self._throttle_log("lane: LOST (bad moments)")
                        return

                offset_px = float(cx - (w // 2))

                lane.in_lane = True
                lane.offset_px = offset_px
                lane.offset_norm = float(offset_px) / float(max(w, 1)) * 2.0

                # quality: contour area vs ROI area (기존 로직 유지)
                roi_area = float(w * max(1, (h - crop_h_start)))
                lane.quality = float(min(1.0, area / (roi_area * 0.10 + 1e-6)))

                # ✅ angle: contour mask 기반
                lane.angle = float(self._estimate_angle_rad(mask_c))

                self.pub_lane.publish(lane)
                self._throttle_log(
                    f"lane: off_px={offset_px:.1f} off_norm={lane.offset_norm:+.3f} "
                    f"ang_deg={math.degrees(lane.angle):+.1f} q={lane.quality:.2f}"
                )

                if bool(self.get_parameter('show_debug').value):
                    dbg = crop.copy()
                    cv2.drawContours(dbg, [c], -1, (0, 0, 255), 2)
                    cv2.line(dbg, (cx, 0), (cx, dbg.shape[0] - 1), (255, 0, 0), 1)
                    cv2.putText(
                        dbg,
                        f"off={lane.offset_norm:+.3f} ang={math.degrees(lane.angle):+.1f}deg q={lane.quality:.2f}",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

                    cv2.imshow("lane_crop", dbg)
                    cv2.imshow("lane_mask", mask)
                    cv2.imshow("lane_mask_contour", mask_c)
                    cv2.waitKey(1)
                return

        # no lane (publish defaults)
        self.pub_lane.publish(lane)
        self._throttle_log("lane: LOST")


def main():
    rclpy.init()
    node = LaneNode()
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
