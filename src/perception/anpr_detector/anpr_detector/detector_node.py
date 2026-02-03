#!/usr/bin/env python3
"""
detector_node.py - ANPR + 장애물 검출 노드

기능:
- 카메라 이미지에서 번호판/스티커 검출
- 장애물 검출 (사람, 박스, 콘)
- OCR로 번호판 텍스트 인식
- 결과 퍼블리시

토픽:
  Subscribe:
    /cam_front/image_raw: sensor_msgs/Image
  Publish:
    /perception/anpr/detections: robot_interfaces/DetectionArray (전체 검출 결과)
    /perception/anpr/plate: std_msgs/String (인식된 번호판 - 호환용)
    /perception/anpr/sticker: std_msgs/String (스티커 유무 - 호환용)
    /perception/anpr/debug_image: sensor_msgs/Image (디버그용)
"""

import sys
import os

# balemaleAI 경로 추가 및 작업 디렉토리 변경 (상대 경로 모델 로드를 위해)
BALEMALE_AI_PATH = '/home/a102/balemaleAI'
sys.path.insert(0, BALEMALE_AI_PATH)
os.chdir(BALEMALE_AI_PATH)

import cv2
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from robot_interfaces.msg import Detection, DetectionArray

# balemaleAI 모듈 import
from core.detector_factory import DetectorFactory
from core.ocr_factory import OCRFactory
from vision.preprocessor import Preprocessor
from vision.logic import RecognitionLogic


class ANPRDetectorNode(Node):
    # 클래스 ID 매핑
    CLASS_PLATE = 0
    CLASS_STICKER = 1
    CLASS_PERSON = 2
    CLASS_BOX = 3
    CLASS_CONE = 4

    CLASS_NAMES = {
        0: 'plate',
        1: 'sticker',
        2: 'person',
        3: 'box',
        4: 'cone',
    }

    def __init__(self):
        super().__init__('anpr_detector')

        # Parameters
        self.declare_parameter('config_path', '/home/a102/balemaleAI/config.yaml')
        self.declare_parameter('image_topic', '/cam_front/image_raw')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_debug_window', False)

        # Load parameters
        config_path = self.get_parameter('config_path').value
        image_topic = self.get_parameter('image_topic').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.show_debug_window = self.get_parameter('show_debug_window').value

        # Load config
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Initialize engines (balemaleAI)
        self.detector = DetectorFactory.create_detector(self.config)
        self.ocr_engine = OCRFactory.create_ocr(self.config)
        self.preprocessor = Preprocessor(self.config)
        self.logic = RecognitionLogic(self.config)

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers - 새로운 DetectionArray
        self.pub_detections = self.create_publisher(
            DetectionArray, '/perception/anpr/detections', 10
        )

        # Publishers - 기존 호환용
        self.pub_plate = self.create_publisher(
            String, '/perception/anpr/plate', 10
        )
        self.pub_sticker = self.create_publisher(
            String, '/perception/anpr/sticker', 10
        )

        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/perception/anpr/debug_image', 10
            )

        # Subscriber
        self.sub_image = self.create_subscription(
            Image, image_topic, self._image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'ANPRDetectorNode started. Subscribing to: {image_topic}'
        )

    def _image_callback(self, msg: Image):
        """이미지 수신 콜백 - 메인 추론 로직"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # A. 전처리 (ROI 등)
        processed_frame = self.preprocessor.run(frame)

        # B. 객체 검출 (Plate, Sticker, Person, Box, Cone)
        detections = self.detector.detect(processed_frame)

        # 클래스별 분리
        plates = [d for d in detections if d['class'] == self.CLASS_PLATE]
        stickers = [d for d in detections if d['class'] == self.CLASS_STICKER]
        obstacles = [d for d in detections if d['class'] in (
            self.CLASS_PERSON, self.CLASS_BOX, self.CLASS_CONE
        )]

        # DetectionArray 메시지 생성
        det_array_msg = DetectionArray()
        det_array_msg.header = msg.header
        det_array_msg.num_plates = len(plates)
        det_array_msg.num_obstacles = len(obstacles)

        plate_text = None
        has_sticker_now = False

        if plates:
            # C. 가장 큰 번호판을 타겟으로 선정
            best_plate = max(
                plates,
                key=lambda p: (p['box'][2] - p['box'][0]) * (p['box'][3] - p['box'][1])
            )

            # 번호판 중심점 계산
            p_center = np.array([
                (best_plate['box'][0] + best_plate['box'][2]) / 2,
                (best_plate['box'][1] + best_plate['box'][3]) / 2
            ])

            # D. 스티커 유무 확인
            has_sticker_now = any(
                np.linalg.norm(p_center - np.array([
                    (s['box'][0] + s['box'][2]) / 2,
                    (s['box'][1] + s['box'][3]) / 2
                ])) < 700
                for s in stickers
            )

            # E. OCR
            plate_img = self.preprocessor.crop_plate(processed_frame, best_plate['box'])
            plate_text = self.ocr_engine.recognize(plate_img)

            # F. 투표 로직 (확정용)
            final_plate, final_sticker = self.logic.update(
                plate_text, best_plate['confidence'], has_sticker_now
            )

            # 매 프레임 OCR 결과 퍼블리시 (plate_text가 있으면)
            if plate_text:
                self.get_logger().info(
                    f'Plate: {plate_text}, Sticker: {has_sticker_now}'
                )

                # 기존 호환 메시지 퍼블리시
                plate_msg = String()
                plate_msg.data = plate_text
                self.pub_plate.publish(plate_msg)

                sticker_msg = String()
                sticker_msg.data = str(has_sticker_now)
                self.pub_sticker.publish(sticker_msg)

        # 모든 검출 결과를 Detection 메시지로 변환
        for det in detections:
            detection_msg = Detection()
            detection_msg.class_id = det['class']
            detection_msg.class_name = self.CLASS_NAMES.get(det['class'], 'unknown')
            detection_msg.confidence = float(det['confidence'])
            detection_msg.x1 = int(det['box'][0])
            detection_msg.y1 = int(det['box'][1])
            detection_msg.x2 = int(det['box'][2])
            detection_msg.y2 = int(det['box'][3])

            # 번호판인 경우 OCR 결과 추가
            if det['class'] == self.CLASS_PLATE and plate_text:
                detection_msg.text = plate_text
                detection_msg.has_sticker = has_sticker_now

            det_array_msg.detections.append(detection_msg)

        # DetectionArray 퍼블리시
        self.pub_detections.publish(det_array_msg)

        # 장애물 로깅
        if obstacles:
            obstacle_summary = ', '.join([
                f"{self.CLASS_NAMES[o['class']]}({o['confidence']:.2f})"
                for o in obstacles
            ])
            self.get_logger().info(f'Obstacles: {obstacle_summary}')

        # 디버그 이미지
        if self.publish_debug or self.show_debug_window:
            debug_image = self._draw_detections(frame, detections)

            if self.publish_debug:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                    debug_msg.header = msg.header
                    self.pub_debug.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Debug image error: {e}')

            if self.show_debug_window:
                cv2.imshow('ANPR Detector', debug_image)
                cv2.waitKey(1)

    def _draw_detections(self, frame, detections):
        """검출 결과를 이미지에 그리기"""
        result = frame.copy()
        colors = {
            0: (0, 255, 0),    # Plate - Green
            1: (255, 0, 0),    # Sticker - Blue
            2: (0, 0, 255),    # Person - Red
            3: (255, 255, 0),  # Box - Cyan
            4: (0, 165, 255),  # Cone - Orange
        }

        for det in detections:
            x1, y1, x2, y2 = map(int, det['box'])
            cls = det['class']
            conf = det['confidence']
            color = colors.get(cls, (255, 255, 255))
            label = self.CLASS_NAMES.get(cls, f'Class{cls}')

            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                result, f'{label} {conf:.2f}',
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 2
            )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = ANPRDetectorNode()

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
