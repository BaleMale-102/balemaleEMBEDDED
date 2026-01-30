#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rc_interfaces.msg import ParkingLineStatus


@dataclass
class LineResult:
    valid: bool = False
    offset_px: float = 0.0
    offset_norm: float = 0.0
    angle: float = 0.0
    quality: float = 0.0
    # debug
    x1: int = 0
    y1: int = 0
    x2: int = 0
    y2: int = 0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class ParkingLineNode(Node):
    """
    parking_line_node
    - Input : sensor_msgs/Image (param: image_topic)
    - Output: rc_interfaces/ParkingLineStatus -> /perception/parking_line
    """

    def __init__(self):
        super().__init__('parking_line_node')

        # ===================== Params =====================
        self.declare_parameter('image_topic', '/cam_side/image_raw')
        self.declare_parameter('output_topic', '/perception/parking_line')
        self.declare_parameter('show_debug', False)

        # ROI (비율) - 
        self.declare_parameter('roi_y_min_ratio', 0.45)  # 0~1
        self.declare_parameter('roi_y_max_ratio', 1.00)

        # Canny / Hough
        self.declare_parameter('canny1', 50)
        self.declare_parameter('canny2', 150)
        self.declare_parameter('hough_threshold', 40)
        self.declare_parameter('min_line_length', 40)
        self.declare_parameter('max_line_gap', 20)

        # scoring
        self.declare_parameter('min_quality', 0.15)  # valid 판정 기준
        self.declare_parameter('prefer_vertical', True)  # "수직 라인"을 기준선으로 선호(기본)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.show_debug = self.get_parameter('show_debug').get_parameter_value().bool_value

        self.bridge = CvBridge()

        # ===================== Pub/Sub =====================
        self.sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            ParkingLineStatus, self.output_topic, 10
        )

        self._last_log_t = 0.0
        self.get_logger().info(
            f'parking_line_node start. image_topic={self.image_topic}, output_topic={self.output_topic}, show_debug={self.show_debug}'
        )

    def on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        res = self.detect_parking_line(img)

        out = ParkingLineStatus()
        out.header = msg.header  # stamp 유지 (perception 규칙)
        out.valid = bool(res.valid)
        out.offset_px = float(res.offset_px)
        out.offset_norm = float(res.offset_norm)
        out.angle = float(res.angle)
        out.quality = float(res.quality)
        self.pub.publish(out)

        # throttled log
        now = time.time()
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            # self.get_logger().info(
            #     f'valid={out.valid} off_px={out.offset_px:+.1f} off_n={out.offset_norm:+.3f} ang={out.angle:+.3f} q={out.quality:.2f}'
            # )

        if self.show_debug:
            dbg = img.copy()
            h, w = dbg.shape[:2]

            y0 = int(h * self.get_parameter('roi_y_min_ratio').value)
            y1 = int(h * self.get_parameter('roi_y_max_ratio').value)
            cv2.rectangle(dbg, (0, y0), (w - 1, y1 - 1), (255, 255, 0), 2)

            if res.valid:
                cv2.line(dbg, (res.x1, res.y1), (res.x2, res.y2), (0, 255, 0), 3)
                cv2.circle(dbg, (w // 2, int(h * 0.85)), 6, (0, 0, 255), -1)

            cv2.putText(
                dbg,
                f'valid={out.valid} off_px={out.offset_px:+.1f} ang={out.angle:+.2f} q={out.quality:.2f}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
            cv2.imshow('parking_line_debug', dbg)
            cv2.waitKey(1)

    def detect_parking_line(self, bgr: np.ndarray) -> LineResult:
        h, w = bgr.shape[:2]

        # ROI crop
        y_min = int(h * float(self.get_parameter('roi_y_min_ratio').value))
        y_max = int(h * float(self.get_parameter('roi_y_max_ratio').value))
        y_min = clamp(y_min, 0, h - 2)
        y_max = clamp(y_max, y_min + 1, h)

        roi = bgr[y_min:y_max, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        c1 = int(self.get_parameter('canny1').value)
        c2 = int(self.get_parameter('canny2').value)
        edges = cv2.Canny(gray, c1, c2)

        # Hough lines
        thr = int(self.get_parameter('hough_threshold').value)
        min_len = int(self.get_parameter('min_line_length').value)
        gap = int(self.get_parameter('max_line_gap').value)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=thr,
            minLineLength=min_len,
            maxLineGap=gap
        )

        if lines is None or len(lines) == 0:
            return LineResult(valid=False, quality=0.0)

        prefer_vertical = bool(self.get_parameter('prefer_vertical').value)

        # 후보 라인 스코어링: 길이 + (수직 선호면) 수직에 가까울수록 가산
        best = None
        best_score = -1.0
        best_meta = None

        for l in lines[:, 0, :]:
            x1, y1, x2, y2 = map(int, l)
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < 1e-3:
                continue

            # 라인 각도: atan2(dy, dx) -> [-pi, pi]
            theta = math.atan2(dy, dx)

            # "수직" 기준 각도(위아래): dx=0일 때 가까움
            # 수직과의 차이: theta가 +-pi/2 근처면 수직
            # vertical_error = 0이면 완전 수직
            vertical_error = abs(abs(theta) - (math.pi / 2.0))  # [0..pi/2]
            vertical_score = 1.0 - clamp(vertical_error / (math.pi / 2.0), 0.0, 1.0)  # 1이 좋음

            if prefer_vertical:
                score = (0.7 * (length / w)) + (0.3 * vertical_score)
            else:
                score = (length / w)

            if score > best_score:
                best_score = score
                best = (x1, y1, x2, y2)
                best_meta = (theta, length, vertical_score)

        if best is None:
            return LineResult(valid=False, quality=0.0)

        x1, y1, x2, y2 = best
        theta, length, vertical_score = best_meta

        # ROI 좌표 -> 원본 이미지 좌표로 변환
        y1_img = y1 + y_min
        y2_img = y2 + y_min
        x1_img = x1
        x2_img = x2

        # offset 계산: 기준 y_ref에서 라인의 x 위치와 이미지 중심 비교
        # y_ref는 하단 근처(0.85h)
        y_ref = int(h * 0.85)
        # 라인 방정식으로 x(y_ref) 구하기
        # (x, y) = (x1, y1) + t*(dx, dy)
        dx = (x2_img - x1_img)
        dy = (y2_img - y1_img)

        # dy가 0이면(수평 라인) y로 x를 구하기 곤란 -> x는 중간값으로 처리
        if abs(dy) < 1e-6:
            x_at_y = (x1_img + x2_img) * 0.5
        else:
            t = (y_ref - y1_img) / float(dy)
            x_at_y = x1_img + t * float(dx)

        cx = (w - 1) * 0.5
        offset_px = float(x_at_y - cx)
        offset_norm = float(offset_px / cx) if cx > 1 else 0.0
        offset_norm = clamp(offset_norm, -1.0, 1.0)

        # angle 정의: "수직 기준선"에서 얼마나 기울었는지로 정규화
        # theta가 pi/2(수직)이면 angle=0.
        # angle 범위를 [-pi/2, +pi/2] 근처로 맞춤
        angle = float((math.pi / 2.0) - theta)
        # wrap to [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        # 필요 이상 커지면 clamp (주차정렬에선 +-1rad 정도면 충분)
        angle = float(clamp(angle, -1.57, 1.57))

        # quality: 길이/화면폭 기반 + (수직 선호면) 수직 점수
        q_len = clamp(length / float(w), 0.0, 1.0)
        q = 0.7 * q_len + 0.3 * float(vertical_score) if prefer_vertical else q_len
        q = float(clamp(q, 0.0, 1.0))

        valid = q >= float(self.get_parameter('min_quality').value)

        return LineResult(
            valid=valid,
            offset_px=offset_px,
            offset_norm=offset_norm,
            angle=angle,
            quality=q,
            x1=int(x1_img), y1=int(y1_img), x2=int(x2_img), y2=int(y2_img)
        )


def main():
    rclpy.init()
    node = ParkingLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_debug:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

