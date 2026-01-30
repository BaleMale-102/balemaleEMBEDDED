#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rc_interfaces.msg import LaneStatus

import cv2
import numpy as np
from cv_bridge import CvBridge
import math
import time


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LaneNode(Node):
    def __init__(self):
        super().__init__('lane_node')
        self.bridge = CvBridge()

        # ===== Params =====
        self.declare_parameter('image_topic', '/cam_bottom/image_raw')
        self.declare_parameter('show_debug', True)

        self.declare_parameter('crop_h_start_ratio', 0.5)
        self.declare_parameter('black_v_max', 60)

        self.declare_parameter('min_contour_area', 300)
        self.declare_parameter('angle_max_deg', 30.0)

        self.declare_parameter('frame_id', 'camera_bottom')

        # pub/sub
        self.pub_lane = self.create_publisher(LaneStatus, '/perception/lane', 10)
        topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(Image, topic, self.cb_img, qos_profile_sensor_data)

        self.frame = None
        self.timer = self.create_timer(0.03, self.on_timer)

        self.get_logger().info("Simple black-line lane_node started")

    def cb_img(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            self.frame = None

    def on_timer(self):
        if self.frame is None:
            return

        frame = self.frame.copy()
        h, w, _ = frame.shape

        # ===== ROI =====
        ratio = self.get_parameter('crop_h_start_ratio').value
        y0 = int(h * ratio)
        crop = frame[y0:h, :]

        # ===== Black mask =====
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        v_max = int(self.get_parameter('black_v_max').value)
        lower = np.array([0, 0, 0])
        upper = np.array([180, 255, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # ===== Find contour =====
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lane = LaneStatus()
        lane.header.stamp = self.get_clock().now().to_msg()
        lane.header.frame_id = self.get_parameter('frame_id').value

        lane.in_lane = False
        lane.offset_px = 0.0
        lane.offset_norm = 0.0
        lane.angle = 0.0
        lane.quality = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > self.get_parameter('min_contour_area').value:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])

                    # offset
                    offset_px = cx - (w // 2)
                    lane.offset_px = float(offset_px)
                    lane.offset_norm = float(offset_px) / (w / 2)

                    # angle
                    pts = c.reshape(-1, 2).astype(np.float32)
                    vx, vy, _, _ = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)

                    # 위쪽 기준 각도
                    if vy > 0:
                        vx = -vx
                        vy = -vy

                    angle = math.atan2(vx, -vy)
                    ang_max = math.radians(self.get_parameter('angle_max_deg').value)
                    angle = clamp(angle, -ang_max, ang_max)

                    lane.angle = float(angle)
                    lane.in_lane = True
                    lane.quality = float(min(1.0, area / (crop.shape[0] * crop.shape[1])))

                    self.pub_lane.publish(lane)

                    if self.get_parameter('show_debug').value:
                        dbg = crop.copy()
                        cv2.drawContours(dbg, [c], -1, (0, 0, 255), 2)
                        cv2.line(dbg, (cx, 0), (cx, dbg.shape[0]), (255, 0, 0), 2)
                        cv2.putText(dbg,
                                    f"off={lane.offset_norm:+.3f} ang={math.degrees(angle):+.1f}",
                                    (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8, (0, 255, 0), 2)
                        cv2.imshow("lane_crop", dbg)
                        cv2.imshow("lane_mask", mask)
                        cv2.waitKey(1)
                    return

        # no lane
        self.pub_lane.publish(lane)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
