#!/usr/bin/env python3
"""
RoArm-M2-S Real-time Gesture Control System
실시간 제스처 제어 시스템 메인 애플리케이션

모든 모듈을 통합한 완전한 손 제스처 제어 시스템
"""

import cv2
import numpy as np
import time
import json
import threading
import argparse
from typing import Optional, Dict, Any
from dataclasses import dataclass

from gesture_detector import AdvancedGestureDetector, GestureType, GestureResult
from roarm_controller import RoArmController
from gesture_mapping import GestureActionMapper
from continuous_hand_control import ContinuousHandController, ControlMode

@dataclass
class SystemConfig:
    """시스템 설정"""
    robot_ip: str = "192.168.4.1"
    camera_index: int = 0
    capture_width: int = 640  # 실제 캡처 해상도 (처리용)
    capture_height: int = 480  # 실제 캡처 해상도 (처리용)
    window_width: int = 800  # 디스플레이 창 크기 (표시용)
    window_height: int = 600  # 디스플레이 창 크기 (표시용)
    fps_limit: int = 60  # FPS 증가로 부드러운 처리
    debug_mode: bool = False
    enable_recording: bool = False
    recording_path: str = "./recordings/"
    control_mode: ControlMode = ControlMode.POSITION_CONTINUOUS  # 연속 제어 모드

class GestureControlGUI:
    """제스처 제어 GUI 클래스"""

    def __init__(self, config: SystemConfig):
        self.config = config
        self.gesture_detector = AdvancedGestureDetector()
        self.robot_controller = RoArmController(config.robot_ip)
        self.gesture_mapper = GestureActionMapper(self.robot_controller)

        # 연속 제어 시스템 추가
        self.continuous_controller = ContinuousHandController(config.robot_ip)
        self.continuous_controller.debug_mode = config.debug_mode

        # 시스템 상태
        self.running = False
        self.paused = False
        self.recording = False

        # 통계
        self.frame_count = 0
        self.fps = 0
        self.last_fps_update = time.time()

        # UI 업데이트 최적화
        self.ui_update_counter = 0
        self.ui_update_interval = 3  # 3프레임마다 UI 업데이트

        # UI 상태
        self.show_landmarks = True
        self.show_debug_info = True
        self.current_gesture = GestureType.IDLE
        self.gesture_confidence = 0.0

    def initialize_system(self) -> bool:
        """시스템 초기화"""
        print("🚀 RoArm-M2-S 제스처 제어 시스템 초기화 중...")

        # 카메라 초기화
        self.cap = cv2.VideoCapture(self.config.camera_index)
        if not self.cap.isOpened():
            print("❌ 카메라 초기화 실패")
            return False

        # 낮은 해상도로 캡처 (FPS 향상)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.capture_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.capture_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.config.fps_limit)
        # 버퍼 크기 줄여서 지연 감소
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 로봇 연결
        if not self.robot_controller.connect():
            print("⚠️ 로봇팔 연결 실패 - 시뮬레이션 모드로 실행")
            self.robot_controller.is_connected = True

        # 연속 제어 시스템 연결
        if not self.continuous_controller.connect():
            print("⚠️ 연속 제어 시스템 연결 실패")

        # 홈 포지션으로 초기화 (모든 모드에서 동일)
        print("🏠 홈 포지션으로 이동 중...")
        self.robot_controller.home_position()
        time.sleep(1)  # 홈 포지션 이동 대기

        print("✅ 시스템 초기화 완료!")
        return True

    def create_info_panel(self, frame: np.ndarray) -> np.ndarray:
        """정보 패널 생성"""
        h, w = frame.shape[:2]
        panel_height = 200
        panel = np.zeros((panel_height, w, 3), dtype=np.uint8)

        # 배경색
        panel[:] = (40, 40, 40)

        # 제목
        cv2.putText(panel, "RoArm-M2-S Gesture Control", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 현재 제스처
        gesture_text = f"Gesture: {self.current_gesture.value}"
        gesture_color = (0, 255, 0) if self.current_gesture != GestureType.IDLE else (128, 128, 128)
        cv2.putText(panel, gesture_text, (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, gesture_color, 2)

        # 신뢰도
        confidence_text = f"Confidence: {self.gesture_confidence:.2f}"
        cv2.putText(panel, confidence_text, (10, 80),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # 로봇 상태
        robot_status = self.robot_controller.get_status()
        pos = robot_status["position"]
        position_text = f"Position: ({pos['x']:.0f}, {pos['y']:.0f}, {pos['z']:.0f})"
        cv2.putText(panel, position_text, (10, 105),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 그리퍼 상태
        gripper_status = "Closed" if robot_status["gripper_closed"] else "Open"
        gripper_color = (0, 0, 255) if robot_status["gripper_closed"] else (0, 255, 0)
        gripper_text = f"Gripper: {gripper_status}"
        cv2.putText(panel, gripper_text, (10, 130),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, gripper_color, 1)

        # 시스템 상태
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(panel, fps_text, (10, 155),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 연결 상태
        connection_status = "Connected" if self.robot_controller.is_connected else "Disconnected"
        connection_color = (0, 255, 0) if self.robot_controller.is_connected else (0, 0, 255)
        cv2.putText(panel, f"Robot: {connection_status}", (10, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, connection_color, 1)

        # 통계 정보
        stats = self.gesture_mapper.get_statistics()
        if stats["total_actions"] > 0:
            stats_text = f"Actions: {stats['total_actions']} (Success: {stats['success_rate']})"
            cv2.putText(panel, stats_text, (w - 400, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return panel

    def draw_gesture_guide(self, frame: np.ndarray):
        """제스처 가이드 그리기"""
        h, w = frame.shape[:2]

        # 가이드 박스
        guide_x = w - 250
        guide_y = 50
        guide_w = 240
        guide_h = 180

        # 반투명 배경
        overlay = frame.copy()
        cv2.rectangle(overlay, (guide_x, guide_y), (guide_x + guide_w, guide_y + guide_h),
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        # 제목
        cv2.putText(frame, "Gesture Guide", (guide_x + 10, guide_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 제스처 목록
        gestures = [
            ("Pinch: Gripper", (0, 255, 255)),
            ("Open Hand: Home", (0, 255, 0)),
            ("Fist: Emergency Stop", (0, 0, 255)),
            ("Palm Forward: Move Forward", (255, 255, 0)),
            ("Palm Back: Move Backward", (255, 255, 0)),
            ("Palm Left: Move Left", (255, 255, 0)),
            ("Palm Right: Move Right", (255, 255, 0))
        ]

        for i, (text, color) in enumerate(gestures):
            y_pos = guide_y + 50 + i * 18
            cv2.putText(frame, text, (guide_x + 10, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def update_fps(self):
        """FPS 업데이트"""
        self.frame_count += 1
        current_time = time.time()

        if current_time - self.last_fps_update >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_fps_update)
            self.frame_count = 0
            self.last_fps_update = current_time

    def handle_keyboard_input(self, key: int) -> bool:
        """키보드 입력 처리"""
        if key == ord('q') or key == 27:  # Q 또는 ESC
            return False

        elif key == ord(' '):  # 스페이스바 - 일시정지
            self.paused = not self.paused
            status = "일시정지" if self.paused else "재개"
            print(f"시스템 {status}")

        elif key == ord('h'):  # H - 홈 포지션
            self.robot_controller.home_position()
            print("홈 포지션으로 이동")

        elif key == ord('s'):  # S - 안전 위치
            self.robot_controller.safe_position()
            print("안전 위치로 이동")

        elif key == ord('e'):  # E - 비상 정지
            self.robot_controller.emergency_stop()
            print("비상 정지!")

        elif key == ord('l'):  # L - 랜드마크 토글
            self.show_landmarks = not self.show_landmarks
            print(f"랜드마크 표시: {'ON' if self.show_landmarks else 'OFF'}")

        elif key == ord('d'):  # D - 디버그 정보 토글
            self.show_debug_info = not self.show_debug_info
            print(f"디버그 정보: {'ON' if self.show_debug_info else 'OFF'}")

        elif key == ord('r'):  # R - 녹화 토글
            if self.config.enable_recording:
                self.recording = not self.recording
                status = "시작" if self.recording else "중지"
                print(f"녹화 {status}")

        elif key == ord('c'):  # C - 그리퍼 토글
            self.robot_controller.control_gripper(not self.robot_controller.gripper_closed)
            status = "닫힘" if self.robot_controller.gripper_closed else "열림"
            print(f"그리퍼 {status}")

        elif key == ord('m'):  # M - 제어 모드 전환
            if self.config.control_mode == ControlMode.POSITION_CONTINUOUS:
                self.config.control_mode = ControlMode.GESTURE_DISCRETE
                print("📋 제어 모드: 제스처 기반 제어")
            else:
                self.config.control_mode = ControlMode.POSITION_CONTINUOUS
                print("📍 제어 모드: 연속 위치 기반 제어")

        elif key == ord('n'):  # N - 중립 위치
            if self.config.control_mode == ControlMode.POSITION_CONTINUOUS:
                self.continuous_controller.set_neutral_position()
                print("🎯 중립 위치로 이동")

        elif key == ord('i'):  # I - 현재 위치를 중립 위치로 설정
            if self.config.control_mode == ControlMode.POSITION_CONTINUOUS:
                success = self.continuous_controller.calibrate_current_position()
                if success:
                    print("📍 현재 위치가 새로운 중립 위치로 설정됨")
                else:
                    print("❌ 캘리브레이션 실패")

        return True

    def run(self):
        """메인 실행 루프"""
        if not self.initialize_system():
            return

        self.running = True
        print("\n🎮 제스처 제어 시작!")
        print("키보드 단축키:")
        print("  SPACE: 일시정지/재개")
        print("  H: 홈 포지션")
        print("  S: 안전 위치")
        print("  E: 비상 정지")
        print("  C: 그리퍼 토글")
        print("  M: 제어 모드 전환 (제스처 ↔ 연속)")
        print("  N: 중립 위치로 이동")
        print("  I: 현재 위치를 중립 위치로 설정")
        print("  L: 랜드마크 표시 토글")
        print("  D: 디버그 정보 토글")
        print("  Q/ESC: 종료")

        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ 프레임 읽기 실패")
                    break

                # 이미지 전처리
                frame = cv2.flip(frame, 1)  # 좌우 반전

                # 디스플레이용으로 리사이즈 (캡처는 640x480, 표시는 800x600)
                if frame.shape[1] != self.config.window_width or frame.shape[0] != self.config.window_height:
                    display_frame = cv2.resize(frame, (self.config.window_width, self.config.window_height))
                else:
                    display_frame = frame

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 처리는 원본 해상도로

                # FPS 업데이트
                self.update_fps()

                if not self.paused:
                    # 손 인식
                    results = self.gesture_detector.hands.process(rgb_frame)

                    gesture_result = None
                    stable_gesture = None

                    if results.multi_hand_landmarks:
                        # 랜드마크 그리기 (디스플레이 프레임에)
                        if self.show_landmarks:
                            self.gesture_detector.draw_landmarks_and_info(display_frame, results)

                        # 제스처 인식
                        landmarks = self.gesture_detector.extract_landmarks(results)
                        if landmarks:
                            # 제어 모드에 따른 처리
                            if self.config.control_mode == ControlMode.POSITION_CONTINUOUS:
                                # 연속 위치 기반 제어
                                self.continuous_controller.update(landmarks)

                                # 제스처 정보 업데이트 (UI 표시용)
                                gesture_result = self.gesture_detector.analyze_gesture(landmarks)
                                self.current_gesture = gesture_result.gesture_type
                                self.gesture_confidence = gesture_result.confidence

                            else:
                                # 기존 제스처 기반 제어
                                gesture_result = self.gesture_detector.analyze_gesture(landmarks)
                                stable_gesture = self.gesture_detector.detect_gesture(landmarks)

                                self.current_gesture = gesture_result.gesture_type
                                self.gesture_confidence = gesture_result.confidence

                                # 안정화된 제스처가 있으면 실행
                                if stable_gesture and stable_gesture != GestureType.IDLE:
                                    success = self.gesture_mapper.execute_gesture(stable_gesture)
                                    if success and self.config.debug_mode:
                                        print(f"제스처 실행: {stable_gesture.value}")

                # UI 요소 그리기 (최적화: 매 프레임마다 업데이트하지 않음)
                self.ui_update_counter += 1
                if self.ui_update_counter >= self.ui_update_interval:
                    self.ui_update_counter = 0

                    if self.show_debug_info:
                        self.draw_gesture_guide(display_frame)

                    # 정보 패널 생성 및 결합 (디스플레이 프레임 사용)
                    info_panel = self.create_info_panel(display_frame)
                    self.cached_combined_frame = np.vstack([display_frame, info_panel])
                else:
                    # 캐시된 프레임 사용 (성능 향상)
                    if hasattr(self, 'cached_combined_frame'):
                        # 새 프레임만 업데이트
                        frame_height = display_frame.shape[0]
                        self.cached_combined_frame[:frame_height] = display_frame
                        combined_frame = self.cached_combined_frame
                    else:
                        # 첫 번째 프레임은 전체 생성
                        info_panel = self.create_info_panel(display_frame)
                        combined_frame = np.vstack([display_frame, info_panel])
                        self.cached_combined_frame = combined_frame

                # 일시정지 표시
                if self.paused:
                    cv2.putText(combined_frame, "PAUSED", (50, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)

                # 화면 출력
                cv2.imshow('RoArm-M2-S Gesture Control', combined_frame)

                # 키보드 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if not self.handle_keyboard_input(key):
                    break

        except KeyboardInterrupt:
            print("\n⚠️ 사용자 중단")

        finally:
            self.cleanup()

    def cleanup(self):
        """시스템 정리"""
        print("🔧 시스템 정리 중...")

        self.running = False

        # 로봇 안전 위치로 이동
        if self.robot_controller.is_connected:
            print("🏠 안전 위치로 이동 중...")
            self.robot_controller.safe_position()

        # 리소스 해제
        if hasattr(self, 'cap'):
            self.cap.release()

        cv2.destroyAllWindows()

        # 통계 출력
        stats = self.gesture_mapper.get_statistics()
        print(f"\n📊 세션 통계:")
        print(f"  총 동작 수: {stats.get('total_actions', 0)}")
        print(f"  성공률: {stats.get('success_rate', '0%')}")
        print(f"  평균 실행 시간: {stats.get('average_duration', '0s')}")

        print("✅ 시스템 종료 완료")

def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description="RoArm-M2-S Gesture Control System")
    parser.add_argument("--robot-ip", default="192.168.4.1", help="RoArm-M2-S IP 주소")
    parser.add_argument("--camera", type=int, default=0, help="카메라 인덱스")
    parser.add_argument("--width", type=int, default=800, help="화면 너비")
    parser.add_argument("--height", type=int, default=600, help="화면 높이")
    parser.add_argument("--fps", type=int, default=30, help="FPS 제한")
    parser.add_argument("--debug", action="store_true", help="디버그 모드")
    parser.add_argument("--record", action="store_true", help="녹화 활성화")

    args = parser.parse_args()

    # 설정 생성
    config = SystemConfig(
        robot_ip=args.robot_ip,
        camera_index=args.camera,
        capture_width=640,  # 캡처 해상도는 고정
        capture_height=480,  # 캡처 해상도는 고정
        window_width=args.width,
        window_height=args.height,
        fps_limit=args.fps,
        debug_mode=args.debug,
        enable_recording=args.record
    )

    # 시스템 실행
    system = GestureControlGUI(config)
    system.run()

if __name__ == "__main__":
    main()