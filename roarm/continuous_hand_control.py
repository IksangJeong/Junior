#!/usr/bin/env python3
"""
Continuous Hand Position Control System
연속적인 손 위치 기반 로봇팔 제어 시스템

손 위치를 실시간으로 추적하여 로봇팔 관절을 직관적으로 제어
"""

import math
import time
import numpy as np
from typing import Tuple, Dict, Optional, List
from dataclasses import dataclass
from enum import Enum

from gesture_detector import HandLandmarks, GestureType
from roarm_controller import RoArmController

class ControlMode(Enum):
    """제어 모드"""
    POSITION_CONTINUOUS = "position_continuous"  # 위치 기반 연속 제어
    GESTURE_DISCRETE = "gesture_discrete"        # 제스처 기반 개별 제어
    HYBRID = "hybrid"                            # 혼합 모드

@dataclass
class HandPosition:
    """정규화된 손 위치"""
    x: float  # 0.0 (왼쪽) ~ 1.0 (오른쪽)
    y: float  # 0.0 (위) ~ 1.0 (아래)
    z: float  # 0.0 (가까움) ~ 1.0 (멀음)

@dataclass
class JointAngles:
    """관절 각도 (라디안)"""
    base: float      # 베이스 회전 (-π/2 ~ π/2)
    shoulder: float  # 어깨 (0 ~ π)
    elbow: float     # 팔꿈치 (0 ~ π/2)
    hand: float      # 그리퍼 (0 ~ π)

class SmoothingFilter:
    """관절 움직임 스무딩 필터"""

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self.previous_angles: Optional[JointAngles] = None

    def smooth(self, new_angles: JointAngles) -> JointAngles:
        """지수 이동 평균으로 스무딩"""
        if self.previous_angles is None:
            self.previous_angles = new_angles
            return new_angles

        smoothed = JointAngles(
            base=self.alpha * new_angles.base + (1 - self.alpha) * self.previous_angles.base,
            shoulder=self.alpha * new_angles.shoulder + (1 - self.alpha) * self.previous_angles.shoulder,
            elbow=self.alpha * new_angles.elbow + (1 - self.alpha) * self.previous_angles.elbow,
            hand=self.alpha * new_angles.hand + (1 - self.alpha) * self.previous_angles.hand
        )

        self.previous_angles = smoothed
        return smoothed

class FastGripperControl:
    """빠른 그리퍼 제어 - 연속적 제어"""

    def __init__(self, controller: RoArmController):
        self.controller = controller
        self.current_state = False  # 현재 그리퍼 상태
        # 히스테리시스 임계값 (두 개의 다른 임계값)
        self.pinch_start_threshold = 0.035  # 집게 시작 임계값 (엄지와 검지가 거의 붙어야 인식)
        self.pinch_end_threshold = 0.08    # 집게 해제 임계값 (적당히 떨어지면 해제)
        self.command_interval = 0.05  # 명령 전송 간격 (초) - 더 빠른 반응
        self.debug_mode = False      # 디버깅 모드 (연속 제어시 너무 많은 출력 방지)
        self.last_command_time = 0
        self.last_debug_time = 0

    def detect_pinch(self, landmarks: HandLandmarks) -> bool:
        """집게 제스처 히스테리시스 감지"""
        if not landmarks:
            return self.current_state  # 랜드마크 없으면 현재 상태 유지

        # 엄지와 검지 끝 거리 계산
        thumb_tip = landmarks.thumb_tip
        index_tip = landmarks.index_tip

        distance = math.sqrt(
            (thumb_tip[0] - index_tip[0])**2 +
            (thumb_tip[1] - index_tip[1])**2
        )

        # 히스테리시스 로직: 현재 상태에 따라 다른 임계값 적용
        if not self.current_state:
            # 현재 잡지 않고 있는 상태 → 가까워지면 잡기 시작
            new_pinch = distance < self.pinch_start_threshold
        else:
            # 현재 잡고 있는 상태 → 많이 멀어져야만 놓기
            new_pinch = distance < self.pinch_end_threshold

        # 디버그: 1초마다 한 번씩만 출력 (스팸 방지)
        current_time = time.time()
        if self.debug_mode and current_time - self.last_debug_time > 1.0:
            threshold_used = self.pinch_start_threshold if not self.current_state else self.pinch_end_threshold
            pinch_status = "✅ 잡기" if new_pinch else "❌ 놓기"
            print(f"🤏 거리: {distance:.3f} | 임계값: {threshold_used:.3f} | {pinch_status}")
            self.last_debug_time = current_time

        return new_pinch

    def update_gripper(self, landmarks: HandLandmarks) -> bool:
        """그리퍼 연속 제어 - 집게 제스처 동안 계속 닫고, 손 펴면 계속 열기"""
        current_time = time.time()

        # 명령 전송 간격 체크 (너무 자주 보내지 않도록)
        if current_time - self.last_command_time < self.command_interval:
            return False

        # 현재 집게 제스처 상태 확인
        is_pinching = self.detect_pinch(landmarks)

        # 상태 변경 시 로그 출력 (실제 동작 반영)
        if is_pinching != self.current_state:
            # is_pinching=True → 그리퍼 열림, is_pinching=False → 그리퍼 닫힘
            actual_action = "열기" if is_pinching else "닫기"
            print(f"🎯 그리퍼 상태 변경: {actual_action}")
            self.current_state = is_pinching

        # 항상 현재 상태에 맞는 명령 전송 (닫기든 열기든 계속 전송)
        success = self.controller.control_gripper(is_pinching)
        if success:
            self.last_command_time = current_time
            # 상태 변경 시에만 성공 메시지
            if is_pinching != self.controller.gripper_closed:
                # 실제 동작: is_pinching=True → 열림, is_pinching=False → 닫힘
                status = "열림" if is_pinching else "닫힘"
                print(f"✅ 그리퍼 {status}")
        return success

class ContinuousHandController:
    """연속적인 손 위치 기반 로봇팔 제어"""

    def __init__(self, robot_ip: str = "192.168.4.1"):
        self.controller = RoArmController(robot_ip)
        self.smoother = SmoothingFilter(alpha=0.4)
        self.gripper_control = FastGripperControl(self.controller)

        # 제어 설정
        self.control_mode = ControlMode.POSITION_CONTINUOUS
        self.dead_zone = 0.0           # 데드존 제거 (전체 화면 활용)
        self.speed_multiplier = 0.8    # 속도 조절

        # 사용자 요청 반영 각도 범위 (라디안)
        # 베이스: -90도 ~ 90도 (정면이 0도)
        self.base_min = -90 * math.pi / 180    # -90도 (-1.571)
        self.base_max = 90 * math.pi / 180     # 90도 (1.571)

        # 어깨: -70도 ~ 70도 (더 넓은 앞뒤 움직임, 깊이 감도는 유지)
        self.shoulder_min = -70 * math.pi / 180  # -70도 (-1.222)
        self.shoulder_max = 70 * math.pi / 180   # 70도 (1.222)

        # 팔꿈치: -63도 ~ 180도 (실제 하드웨어 -1.11~3.14)
        self.elbow_min = -63 * math.pi / 180   # -63도 (-1.11)
        self.elbow_max = 180 * math.pi / 180   # 180도 (3.14)

        # Z축 깊이 증폭 (민감도 감소로 더 부드러운 제어)
        self.z_amplification = 4

        # 중심 기준점 (손이 이 위치에 있을 때 로봇팔이 중립 상태)
        self.center_position = HandPosition(0.5, 0.5, 0.5)
        # 중립 자세: 새로운 범위에 맞는 중간값
        self.neutral_angles = JointAngles(
            (self.base_min + self.base_max) / 2,        # 베이스 0도 (중간값, 정면)
            (self.shoulder_min + self.shoulder_max) / 2, # 어깨 0도 (중간값)
            (self.elbow_min + self.elbow_max) / 2,      # 팔꿈치 58.5도 (중간값)
            3.14  # 그리퍼 열림 (180도, 3.14 라디안)
        )

        # 상태 추적
        self.last_update_time = time.time()
        self.update_interval = 0.03    # 33Hz 업데이트 (빠른 반응)

    def normalize_hand_position(self, landmarks: HandLandmarks) -> HandPosition:
        """손 랜드마크를 정규화된 위치로 변환"""
        if not landmarks:
            return self.center_position

        # 손바닥 중심 계산
        palm_x = landmarks.palm_center[0]
        palm_y = landmarks.palm_center[1]

        # 손목에서 중지까지의 거리로 Z축(깊이) 추정 (개선된 알고리즘)
        wrist_to_middle = math.sqrt(
            (landmarks.middle_tip[0] - landmarks.wrist[0])**2 +
            (landmarks.middle_tip[1] - landmarks.wrist[1])**2
        )

        # 추가적으로 손바닥 크기로 깊이 보정
        palm_width = abs(landmarks.thumb_tip[0] - landmarks.pinky_tip[0])
        depth_factor = wrist_to_middle * self.z_amplification

        # 손바닥이 클수록(가까울수록) z값이 작아짐
        palm_z = min(1.0, max(0.0, depth_factor - palm_width * 2))

        return HandPosition(palm_x, palm_y, palm_z)

    def position_to_joint_angles(self, hand_pos: HandPosition) -> JointAngles:
        """손 위치를 관절 각도로 변환 - Information.md 하드웨어 명세 기준 매핑"""

        # 화면 좌표(0~1)를 실제 하드웨어 각도 범위로 매핑

        # 베이스 회전: 화면 X축 (0~1) → -90~90도
        # 손이 중앙(x=0.5)일 때 베이스는 정면(0도)
        # 손이 왼쪽(x=0)일 때 베이스는 오른쪽(90도)
        # 손이 오른쪽(x=1)일 때 베이스는 왼쪽(-90도)
        base_angle = self.base_max - (hand_pos.x * (self.base_max - self.base_min))

        # 어깨: 손 크기(Z축) → -70~70도 (더 넓은 앞뒤 움직임, 깊이 감도 유지)
        # Z가 작을수록(손이 가까이/크게) → -70도 (뒤로)
        # Z가 클수록(손이 멀리/작게) → 70도 (앞으로)
        # 범위 확장, 깊이 감도는 Z축 증폭 5로 유지
        shoulder_angle = self.shoulder_min + (hand_pos.z * (self.shoulder_max - self.shoulder_min))

        # 팔꿈치: 화면 Y축 (0~1) → -63~180도 (위아래 움직임)
        # Y=0(위) → -63도(위로 펼침), Y=1(아래) → 180도(아래로 굽힘)
        elbow_angle = self.elbow_min + (hand_pos.y * (self.elbow_max - self.elbow_min))

        # 그리퍼는 별도 제어
        hand_angle = self.neutral_angles.hand

        # 안전 범위 제한 (새로운 범위 내로 제한)
        base_angle = max(self.base_min, min(self.base_max, base_angle))        # -90도 ~ 90도
        shoulder_angle = max(self.shoulder_min, min(self.shoulder_max, shoulder_angle))  # -70도 ~ 70도
        elbow_angle = max(self.elbow_min, min(self.elbow_max, elbow_angle))    # -63도 ~ 180도

        return JointAngles(base_angle, shoulder_angle, elbow_angle, hand_angle)

    def calculate_movement_speed(self, current_pos: HandPosition, previous_pos: HandPosition) -> float:
        """손 움직임 속도 계산"""
        if previous_pos is None:
            return self.speed_multiplier

        # 위치 변화량 계산
        dx = current_pos.x - previous_pos.x
        dy = current_pos.y - previous_pos.y
        velocity = math.sqrt(dx**2 + dy**2)

        # 속도에 따른 로봇팔 속도 조절
        if velocity < 0.01:
            return 0.3  # 느린 움직임
        elif velocity < 0.05:
            return 0.6  # 보통 속도
        else:
            return 1.0  # 빠른 움직임

    def send_continuous_command(self, joint_angles: JointAngles, speed: float = 0.8) -> bool:
        """연속적인 관절 제어 명령 전송"""

        # index.html에서 확인한 관절 제어 명령 (T:102)
        # hand도 포함 - 그리퍼 상태 유지를 위해
        command = {
            "T": 102,
            "base": joint_angles.base,
            "shoulder": joint_angles.shoulder,
            "elbow": joint_angles.elbow,
            "hand": self.controller.current_joints.joint4,  # 현재 그리퍼 상태 유지
            "spd": speed,
            "acc": 0
        }

        return self.controller.send_command(command)

    def update(self, landmarks: HandLandmarks) -> bool:
        """메인 업데이트 함수"""
        current_time = time.time()

        # 업데이트 주기 제한 (성능 최적화)
        if current_time - self.last_update_time < self.update_interval:
            return False

        self.last_update_time = current_time

        # 1. 빠른 그리퍼 제어 (우선 처리)
        self.gripper_control.update_gripper(landmarks)

        # 2. 손 위치 기반 연속 제어
        if self.control_mode == ControlMode.POSITION_CONTINUOUS:
            # 손 위치 정규화
            hand_position = self.normalize_hand_position(landmarks)

            # 관절 각도 계산
            target_angles = self.position_to_joint_angles(hand_position)

            # 스무딩 적용
            smoothed_angles = self.smoother.smooth(target_angles)

            # 속도 계산
            speed = self.speed_multiplier

            # 명령 전송
            success = self.send_continuous_command(smoothed_angles, speed)

            if success and hasattr(self, 'debug_mode') and self.debug_mode:
                print(f"📍 Position: ({hand_position.x:.2f}, {hand_position.y:.2f})")
                print(f"🦾 Angles: B={math.degrees(smoothed_angles.base):.1f}° "
                      f"S={math.degrees(smoothed_angles.shoulder):.1f}° "
                      f"E={math.degrees(smoothed_angles.elbow):.1f}°")

            return success

        return False

    def set_neutral_position(self):
        """중립 위치로 이동"""
        print("🏠 중립 위치로 이동...")
        return self.send_continuous_command(self.neutral_angles, 0.5)

    def calibrate_current_position(self) -> bool:
        """현재 로봇팔 위치를 새로운 중립 위치로 설정"""
        print("📍 현재 위치를 중립 위치로 캘리브레이션 중...")

        # 로봇팔에서 현재 상태 조회
        robot_status = self.controller.get_robot_status()

        if robot_status is None:
            print("❌ 로봇팔 상태 조회 실패")
            return False

        try:
            # 로봇팔의 실제 응답에서 관절 각도 추출
            # 응답 형식에 따라 조정 필요
            if 'angles' in robot_status:
                # 각도 정보가 있는 경우
                angles = robot_status['angles']
                self.neutral_angles = JointAngles(
                    base=angles.get('base', 0),
                    shoulder=angles.get('shoulder', 1.57),
                    elbow=angles.get('elbow', 1.57),
                    hand=angles.get('hand', 3.14)
                )
            else:
                # 기본값 사용하되 현재 컨트롤러의 위치 정보 활용
                self.neutral_angles = JointAngles(
                    base=self.controller.current_joints.joint1,
                    shoulder=self.controller.current_joints.joint2,
                    elbow=self.controller.current_joints.joint3,
                    hand=self.controller.current_joints.joint4
                )

            print(f"✅ 새로운 중립 위치 설정:")
            print(f"   베이스: {math.degrees(self.neutral_angles.base):.1f}°")
            print(f"   어깨: {math.degrees(self.neutral_angles.shoulder):.1f}°")
            print(f"   팔꿈치: {math.degrees(self.neutral_angles.elbow):.1f}°")
            print(f"   그리퍼: {math.degrees(self.neutral_angles.hand):.1f}°")

            return True

        except Exception as e:
            print(f"❌ 캘리브레이션 실패: {e}")
            return False

    def emergency_stop(self):
        """비상 정지"""
        return self.controller.emergency_stop()

    def connect(self) -> bool:
        """로봇팔 연결"""
        return self.controller.connect()

def test_continuous_control():
    """연속 제어 테스트"""
    print("🤖 연속 제어 시스템 테스트")

    controller = ContinuousHandController()

    if not controller.connect():
        print("❌ 로봇팔 연결 실패")
        return

    print("✅ 연결 성공!")

    # 중립 위치로 이동
    controller.set_neutral_position()

    print("테스트 완료")

if __name__ == "__main__":
    test_continuous_control()