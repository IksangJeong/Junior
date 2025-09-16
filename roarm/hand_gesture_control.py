#!/usr/bin/env python3
"""
RoArm-M2-S Hand Gesture Control System
웹캠을 통한 손 제스처 인식으로 로봇팔 제어

제스처 매핑:
- 엄지+검지 붙이기: 그리퍼 닫기/열기
- 손바닥 앞: 로봇팔 앞으로 이동
- 손바닥 뒤: 로봇팔 뒤로 이동
- 손바닥 왼쪽: 로봇팔 왼쪽으로 이동
- 손바닥 오른쪽: 로봇팔 오른쪽으로 이동
"""

import cv2
import mediapipe as mp
import numpy as np
import json
import requests
import time
import math
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass
from enum import Enum

class GestureType(Enum):
    """제스처 타입 정의"""
    PINCH = "pinch"          # 엄지+검지 붙이기
    FORWARD = "forward"      # 앞으로
    BACKWARD = "backward"    # 뒤로
    LEFT = "left"           # 왼쪽
    RIGHT = "right"         # 오른쪽
    IDLE = "idle"           # 대기

@dataclass
class HandLandmarks:
    """손 랜드마크 데이터 클래스"""
    thumb_tip: Tuple[float, float]
    index_tip: Tuple[float, float]
    palm_center: Tuple[float, float]
    wrist: Tuple[float, float]

class HandGestureDetector:
    """손 제스처 인식 클래스"""

    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

    def extract_landmarks(self, results) -> Optional[HandLandmarks]:
        """손 랜드마크 추출"""
        if not results.multi_hand_landmarks:
            return None

        hand_landmarks = results.multi_hand_landmarks[0]

        # 주요 랜드마크 포인트 추출
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        palm_center = hand_landmarks.landmark[9]
        wrist = hand_landmarks.landmark[0]

        return HandLandmarks(
            thumb_tip=(thumb_tip.x, thumb_tip.y),
            index_tip=(index_tip.x, index_tip.y),
            palm_center=(palm_center.x, palm_center.y),
            wrist=(wrist.x, wrist.y)
        )

    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """두 점 사이의 거리 계산"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def calculate_angle(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """두 점 사이의 각도 계산 (라디안)"""
        return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    def detect_gesture(self, landmarks: HandLandmarks) -> GestureType:
        """제스처 인식"""
        # 엄지와 검지 끝점 사이의 거리
        pinch_distance = self.calculate_distance(landmarks.thumb_tip, landmarks.index_tip)

        # 핀치 제스처 감지 (임계값: 0.05)
        if pinch_distance < 0.05:
            return GestureType.PINCH

        # 손바닥의 방향 감지
        palm_to_wrist_angle = self.calculate_angle(landmarks.wrist, landmarks.palm_center)

        # 각도를 도 단위로 변환
        angle_degrees = math.degrees(palm_to_wrist_angle)

        # 방향 판정 (각도 기준)
        if -45 <= angle_degrees <= 45:  # 오른쪽
            return GestureType.RIGHT
        elif 45 < angle_degrees <= 135:  # 앞쪽
            return GestureType.FORWARD
        elif 135 < angle_degrees or angle_degrees <= -135:  # 왼쪽
            return GestureType.LEFT
        else:  # -135 < angle_degrees < -45, 뒤쪽
            return GestureType.BACKWARD

        return GestureType.IDLE

class RoArmController:
    """RoArm-M2-S 제어 클래스"""

    def __init__(self, robot_ip: str = "192.168.4.1", port: int = 80):
        self.robot_ip = robot_ip
        self.port = port
        self.base_url = f"http://{robot_ip}:{port}"
        self.current_position = {"x": 200, "y": 0, "z": 150}  # 초기 위치
        self.gripper_closed = False
        self.move_step = 20  # 이동 단위 (mm)

    def send_command(self, command: Dict[str, Any]) -> bool:
        """로봇팔에 JSON 명령 전송"""
        try:
            url = f"{self.base_url}/json"
            headers = {'Content-Type': 'application/json'}

            response = requests.post(url, json=command, headers=headers, timeout=5)
            return response.status_code == 200
        except Exception as e:
            print(f"명령 전송 실패: {e}")
            return False

    def move_to_position(self, x: int, y: int, z: int, time_ms: int = 1000) -> bool:
        """지정된 좌표로 이동"""
        command = {
            "T": 2,  # 좌표 제어 모드
            "x": x,
            "y": y,
            "z": z,
            "time": time_ms
        }

        if self.send_command(command):
            self.current_position = {"x": x, "y": y, "z": z}
            return True
        return False

    def control_gripper(self, close: bool) -> bool:
        """그리퍼 제어"""
        # 그리퍼는 joint4로 제어 (0: 열림, 90: 닫힘)
        gripper_angle = 90 if close else 0

        command = {
            "T": 1,  # 관절 제어 모드
            "joint4": gripper_angle,
            "time": 500
        }

        if self.send_command(command):
            self.gripper_closed = close
            return True
        return False

    def move_forward(self) -> bool:
        """앞으로 이동"""
        new_x = min(self.current_position["x"] + self.move_step, 300)
        return self.move_to_position(new_x, self.current_position["y"], self.current_position["z"])

    def move_backward(self) -> bool:
        """뒤로 이동"""
        new_x = max(self.current_position["x"] - self.move_step, 100)
        return self.move_to_position(new_x, self.current_position["y"], self.current_position["z"])

    def move_left(self) -> bool:
        """왼쪽으로 이동"""
        new_y = max(self.current_position["y"] - self.move_step, -200)
        return self.move_to_position(self.current_position["x"], new_y, self.current_position["z"])

    def move_right(self) -> bool:
        """오른쪽으로 이동"""
        new_y = min(self.current_position["y"] + self.move_step, 200)
        return self.move_to_position(self.current_position["x"], new_y, self.current_position["z"])

    def home_position(self) -> bool:
        """홈 포지션으로 이동"""
        return self.move_to_position(200, 0, 150, 2000)

class GestureControlSystem:
    """제스처 제어 시스템 메인 클래스"""

    def __init__(self, robot_ip: str = "192.168.4.1"):
        self.detector = HandGestureDetector()
        self.controller = RoArmController(robot_ip)
        self.last_gesture = GestureType.IDLE
        self.gesture_start_time = time.time()
        self.gesture_delay = 1.0  # 제스처 실행 간격 (초)

    def process_gesture(self, gesture: GestureType) -> bool:
        """제스처 처리"""
        current_time = time.time()

        # 같은 제스처가 연속으로 감지된 경우 지연 시간 체크
        if gesture == self.last_gesture and current_time - self.gesture_start_time < self.gesture_delay:
            return False

        # 새로운 제스처이거나 지연 시간이 지난 경우
        if gesture != self.last_gesture or current_time - self.gesture_start_time >= self.gesture_delay:
            self.last_gesture = gesture
            self.gesture_start_time = current_time

            # 제스처에 따른 로봇팔 제어
            if gesture == GestureType.PINCH:
                return self.controller.control_gripper(not self.controller.gripper_closed)
            elif gesture == GestureType.FORWARD:
                return self.controller.move_forward()
            elif gesture == GestureType.BACKWARD:
                return self.controller.move_backward()
            elif gesture == GestureType.LEFT:
                return self.controller.move_left()
            elif gesture == GestureType.RIGHT:
                return self.controller.move_right()

        return False

    def run(self):
        """메인 실행 루프"""
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("RoArm-M2-S 제스처 제어 시작!")
        print("제스처 가이드:")
        print("- 엄지+검지 붙이기: 그리퍼 열기/닫기")
        print("- 손바닥 앞: 앞으로 이동")
        print("- 손바닥 뒤: 뒤로 이동")
        print("- 손바닥 왼쪽: 왼쪽으로 이동")
        print("- 손바닥 오른쪽: 오른쪽으로 이동")
        print("- 'h' 키: 홈 포지션")
        print("- 'q' 키: 종료")

        # 홈 포지션으로 초기화
        self.controller.home_position()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # 이미지 전처리
                frame = cv2.flip(frame, 1)  # 좌우 반전
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # 손 인식
                results = self.detector.hands.process(rgb_frame)

                gesture = GestureType.IDLE
                if results.multi_hand_landmarks:
                    # 손 랜드마크 그리기
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.detector.mp_draw.draw_landmarks(
                            frame, hand_landmarks, self.detector.mp_hands.HAND_CONNECTIONS)

                    # 제스처 인식
                    landmarks = self.detector.extract_landmarks(results)
                    if landmarks:
                        gesture = self.detector.detect_gesture(landmarks)

                        # 제스처 처리
                        if gesture != GestureType.IDLE:
                            success = self.process_gesture(gesture)
                            if success:
                                print(f"제스처 실행: {gesture.value}")

                # 현재 상태 표시
                status_text = f"Gesture: {gesture.value}"
                position_text = f"Position: ({self.controller.current_position['x']}, {self.controller.current_position['y']}, {self.controller.current_position['z']})"
                gripper_text = f"Gripper: {'Closed' if self.controller.gripper_closed else 'Open'}"

                cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, position_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(frame, gripper_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                cv2.imshow('RoArm-M2-S Gesture Control', frame)

                # 키보드 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h'):
                    self.controller.home_position()
                    print("홈 포지션으로 이동")

        except KeyboardInterrupt:
            print("\n프로그램 종료 중...")
        finally:
            cap.release()
            cv2.destroyAllWindows()

def main():
    """메인 함수"""
    # RoArm-M2-S IP 주소 설정 (기본값: AP 모드)
    robot_ip = "192.168.4.1"  # 필요시 실제 IP로 변경

    system = GestureControlSystem(robot_ip)
    system.run()

if __name__ == "__main__":
    main()