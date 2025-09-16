#!/usr/bin/env python3
"""
Advanced Hand Gesture Detection Module
향상된 손 제스처 인식 모듈

주요 기능:
- 정밀한 핀치 제스처 감지
- 손바닥 방향 인식
- 제스처 안정성 필터링
- 실시간 시각적 피드백
"""

import cv2
import mediapipe as mp
import numpy as np
import math
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass
from enum import Enum
import time

class GestureType(Enum):
    """제스처 타입 정의"""
    PINCH = "pinch"          # 엄지+검지 붙이기 (그리퍼 제어)
    FORWARD = "forward"      # 앞으로 (X+)
    BACKWARD = "backward"    # 뒤로 (X-)
    LEFT = "left"           # 왼쪽 (Y-)
    RIGHT = "right"         # 오른쪽 (Y+)
    UP = "up"               # 위로 (Z+)
    DOWN = "down"           # 아래로 (Z-)
    OPEN_HAND = "open"      # 손바닥 펼치기
    FIST = "fist"           # 주먹
    IDLE = "idle"           # 대기

@dataclass
class HandLandmarks:
    """손 랜드마크 데이터 클래스"""
    thumb_tip: Tuple[float, float]      # 엄지 끝
    thumb_ip: Tuple[float, float]       # 엄지 중간
    index_tip: Tuple[float, float]      # 검지 끝
    index_pip: Tuple[float, float]      # 검지 중간
    middle_tip: Tuple[float, float]     # 중지 끝
    ring_tip: Tuple[float, float]       # 약지 끝
    pinky_tip: Tuple[float, float]      # 새끼손가락 끝
    palm_center: Tuple[float, float]    # 손바닥 중심
    wrist: Tuple[float, float]          # 손목

@dataclass
class GestureResult:
    """제스처 인식 결과"""
    gesture_type: GestureType
    confidence: float
    pinch_distance: float
    palm_direction: float
    finger_count: int

class GestureFilter:
    """제스처 안정성 필터"""

    def __init__(self, buffer_size: int = 3, confidence_threshold: float = 0.5):
        self.buffer_size = buffer_size
        self.confidence_threshold = confidence_threshold
        self.gesture_buffer = []

    def add_gesture(self, gesture: GestureType, confidence: float) -> Optional[GestureType]:
        """제스처 버퍼에 추가하고 안정화된 제스처 반환"""
        self.gesture_buffer.append((gesture, confidence))

        # 버퍼 크기 제한
        if len(self.gesture_buffer) > self.buffer_size:
            self.gesture_buffer.pop(0)

        # 충분한 데이터가 모이면 분석
        if len(self.gesture_buffer) >= self.buffer_size:
            return self._analyze_buffer()

        return None

    def _analyze_buffer(self) -> Optional[GestureType]:
        """버퍼 분석하여 안정화된 제스처 결정"""
        gesture_counts = {}
        total_confidence = {}

        for gesture, confidence in self.gesture_buffer:
            if gesture not in gesture_counts:
                gesture_counts[gesture] = 0
                total_confidence[gesture] = 0
            gesture_counts[gesture] += 1
            total_confidence[gesture] += confidence

        # 가장 많이 감지된 제스처 찾기
        most_common_gesture = max(gesture_counts, key=gesture_counts.get)
        avg_confidence = total_confidence[most_common_gesture] / gesture_counts[most_common_gesture]

        # 신뢰도 기준 확인
        if avg_confidence >= self.confidence_threshold:
            return most_common_gesture

        return None

class AdvancedGestureDetector:
    """향상된 손 제스처 인식 클래스"""

    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,  # 낮춤으로 빠른 감지
            min_tracking_confidence=0.3,   # 낮춤으로 부드러운 추적
            model_complexity=0             # 가장 빠른 모델 사용
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.gesture_filter = GestureFilter()

        # 제스처 임계값 설정
        self.pinch_threshold = 0.08  # 증가시켜 더 쉽게 인식
        self.finger_threshold = 0.02

    def extract_landmarks(self, results) -> Optional[HandLandmarks]:
        """손 랜드마크 추출"""
        if not results.multi_hand_landmarks:
            return None

        hand_landmarks = results.multi_hand_landmarks[0]
        landmarks = hand_landmarks.landmark

        return HandLandmarks(
            thumb_tip=(landmarks[4].x, landmarks[4].y),
            thumb_ip=(landmarks[3].x, landmarks[3].y),
            index_tip=(landmarks[8].x, landmarks[8].y),
            index_pip=(landmarks[6].x, landmarks[6].y),
            middle_tip=(landmarks[12].x, landmarks[12].y),
            ring_tip=(landmarks[16].x, landmarks[16].y),
            pinky_tip=(landmarks[20].x, landmarks[20].y),
            palm_center=(landmarks[9].x, landmarks[9].y),
            wrist=(landmarks[0].x, landmarks[0].y)
        )

    def calculate_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """두 점 사이의 거리 계산"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def calculate_angle(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """두 점 사이의 각도 계산 (라디안)"""
        return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    def count_extended_fingers(self, landmarks: HandLandmarks) -> int:
        """펼쳐진 손가락 개수 세기"""
        fingers = []

        # 엄지 (특별 처리)
        if landmarks.thumb_tip[0] > landmarks.thumb_ip[0]:  # 오른손 기준
            fingers.append(1)
        else:
            fingers.append(0)

        # 나머지 손가락들
        finger_tips = [
            landmarks.index_tip, landmarks.middle_tip,
            landmarks.ring_tip, landmarks.pinky_tip
        ]
        finger_pips = [
            landmarks.index_pip, (landmarks.palm_center[0], landmarks.palm_center[1] - 0.05),
            (landmarks.palm_center[0], landmarks.palm_center[1] - 0.03),
            (landmarks.palm_center[0], landmarks.palm_center[1] - 0.02)
        ]

        for tip, pip in zip(finger_tips, finger_pips):
            if tip[1] < pip[1]:  # Y좌표가 작으면 펼쳐진 것
                fingers.append(1)
            else:
                fingers.append(0)

        return sum(fingers)

    def detect_pinch(self, landmarks: HandLandmarks) -> Tuple[bool, float]:
        """핀치 제스처 감지"""
        distance = self.calculate_distance(landmarks.thumb_tip, landmarks.index_tip)
        is_pinch = distance < self.pinch_threshold
        confidence = max(0, 1 - (distance / self.pinch_threshold))
        return is_pinch, confidence

    def detect_palm_direction(self, landmarks: HandLandmarks) -> Tuple[str, float]:
        """손바닥 방향 감지"""
        # 손목에서 손바닥 중심으로의 벡터
        dx = landmarks.palm_center[0] - landmarks.wrist[0]
        dy = landmarks.palm_center[1] - landmarks.wrist[1]

        angle = math.atan2(dy, dx)
        angle_degrees = math.degrees(angle)

        # 각도를 0-360도로 정규화
        if angle_degrees < 0:
            angle_degrees += 360

        # 방향 결정
        if 315 <= angle_degrees or angle_degrees < 45:
            return "right", 0.9
        elif 45 <= angle_degrees < 135:
            return "forward", 0.9
        elif 135 <= angle_degrees < 225:
            return "left", 0.9
        else:  # 225 <= angle_degrees < 315
            return "backward", 0.9

    def analyze_gesture(self, landmarks: HandLandmarks) -> GestureResult:
        """종합적인 제스처 분석"""
        # 핀치 감지
        is_pinch, pinch_confidence = self.detect_pinch(landmarks)
        pinch_distance = self.calculate_distance(landmarks.thumb_tip, landmarks.index_tip)

        # 손가락 개수
        finger_count = self.count_extended_fingers(landmarks)

        # 손바닥 방향
        palm_direction, direction_confidence = self.detect_palm_direction(landmarks)
        palm_angle = math.degrees(self.calculate_angle(landmarks.wrist, landmarks.palm_center))

        # 제스처 결정 로직
        if is_pinch and pinch_confidence > 0.7:
            gesture = GestureType.PINCH
            confidence = pinch_confidence
        elif finger_count == 0:  # 주먹
            gesture = GestureType.FIST
            confidence = 0.8
        elif finger_count >= 4:  # 열린 손
            # 방향에 따라 이동 제스처 결정
            if palm_direction == "forward":
                gesture = GestureType.FORWARD
            elif palm_direction == "backward":
                gesture = GestureType.BACKWARD
            elif palm_direction == "left":
                gesture = GestureType.LEFT
            elif palm_direction == "right":
                gesture = GestureType.RIGHT
            else:
                gesture = GestureType.OPEN_HAND
            confidence = direction_confidence
        else:
            gesture = GestureType.IDLE
            confidence = 0.5

        return GestureResult(
            gesture_type=gesture,
            confidence=confidence,
            pinch_distance=pinch_distance,
            palm_direction=palm_angle,
            finger_count=finger_count
        )

    def detect_gesture(self, landmarks: HandLandmarks) -> Optional[GestureType]:
        """제스처 인식 (필터링 적용)"""
        result = self.analyze_gesture(landmarks)
        return self.gesture_filter.add_gesture(result.gesture_type, result.confidence)

    def draw_landmarks_and_info(self, image, results, gesture_result: Optional[GestureResult] = None):
        """랜드마크와 정보 그리기"""
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 랜드마크 그리기
                self.mp_draw.draw_landmarks(
                    image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                    self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_draw.DrawingSpec(color=(255, 0, 0), thickness=2)
                )

                # 엄지와 검지 끝점 강조
                h, w, _ = image.shape
                thumb_tip = hand_landmarks.landmark[4]
                index_tip = hand_landmarks.landmark[8]

                thumb_pos = (int(thumb_tip.x * w), int(thumb_tip.y * h))
                index_pos = (int(index_tip.x * w), int(index_tip.y * h))

                cv2.circle(image, thumb_pos, 8, (255, 255, 0), -1)
                cv2.circle(image, index_pos, 8, (255, 255, 0), -1)
                cv2.line(image, thumb_pos, index_pos, (255, 255, 0), 2)

        # 제스처 정보 표시
        if gesture_result:
            info_text = [
                f"Gesture: {gesture_result.gesture_type.value}",
                f"Confidence: {gesture_result.confidence:.2f}",
                f"Fingers: {gesture_result.finger_count}",
                f"Pinch Distance: {gesture_result.pinch_distance:.3f}",
                f"Palm Direction: {gesture_result.palm_direction:.1f}°"
            ]

            for i, text in enumerate(info_text):
                y_pos = 30 + i * 25
                cv2.putText(image, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

def test_gesture_detector():
    """제스처 감지기 테스트"""
    detector = AdvancedGestureDetector()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Advanced Gesture Detector Test")
    print("제스처 가이드:")
    print("- 엄지+검지 붙이기: PINCH")
    print("- 손바닥 펼치기 + 방향: FORWARD/BACKWARD/LEFT/RIGHT")
    print("- 주먹: FIST")
    print("- 'q' 키: 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = detector.hands.process(rgb_frame)

        gesture_result = None
        stable_gesture = None

        if results.multi_hand_landmarks:
            landmarks = detector.extract_landmarks(results)
            if landmarks:
                gesture_result = detector.analyze_gesture(landmarks)
                stable_gesture = detector.detect_gesture(landmarks)

        # 시각적 피드백
        detector.draw_landmarks_and_info(frame, results, gesture_result)

        # 안정화된 제스처 표시
        if stable_gesture:
            cv2.putText(frame, f"STABLE: {stable_gesture.value}", (10, 170),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)

        cv2.imshow('Advanced Gesture Detection Test', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_gesture_detector()