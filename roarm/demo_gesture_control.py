#!/usr/bin/env python3
"""
RoArm-M2-S Demo Mode (No Camera Required)
카메라 없이 키보드로 제스처를 시뮬레이션하는 데모 모드

키보드 입력으로 다양한 제스처를 시뮬레이션하여 로봇팔 제어를 테스트할 수 있습니다.
"""

import time
import threading
from typing import Dict, Any
from dataclasses import dataclass

from gesture_detector import GestureType
from roarm_controller import RoArmController
from gesture_mapping import GestureActionMapper

@dataclass
class DemoConfig:
    """데모 설정"""
    robot_ip: str = "192.168.4.1"
    debug_mode: bool = True

class DemoGestureControl:
    """데모 제스처 제어 클래스"""

    def __init__(self, config: DemoConfig):
        self.config = config
        self.robot_controller = RoArmController(config.robot_ip)
        self.gesture_mapper = GestureActionMapper(self.robot_controller)

        # 상태
        self.running = False
        self.current_gesture = GestureType.IDLE

    def initialize_system(self) -> bool:
        """시스템 초기화"""
        print("🚀 RoArm-M2-S 데모 제어 시스템 초기화 중...")

        # 로봇 연결 (시뮬레이션 모드)
        if not self.robot_controller.connect():
            print("⚠️ 로봇팔 연결 실패 - 시뮬레이션 모드로 실행")
            self.robot_controller.is_connected = True

        # 홈 포지션으로 초기화
        print("🏠 홈 포지션으로 이동 중...")
        self.robot_controller.home_position()

        print("✅ 시스템 초기화 완료!")
        return True

    def print_status(self):
        """현재 상태 출력"""
        print("\n" + "="*60)
        print("📊 RoArm-M2-S 현재 상태")
        print("="*60)

        # 로봇 상태
        robot_status = self.robot_controller.get_status()
        pos = robot_status["position"]

        print(f"🤖 로봇 연결: {'✅ 연결됨' if self.robot_controller.is_connected else '❌ 연결 안됨'}")
        print(f"📍 현재 위치: X={pos['x']:.0f}, Y={pos['y']:.0f}, Z={pos['z']:.0f}")
        print(f"🔧 그리퍼: {'🔒 닫힘' if robot_status['gripper_closed'] else '🔓 열림'}")
        print(f"🎯 현재 제스처: {self.current_gesture.value}")

        # 통계
        stats = self.gesture_mapper.get_statistics()
        print(f"📈 총 동작 수: {stats.get('total_actions', 0)}")
        print(f"🎯 성공률: {stats.get('success_rate', '0%')}")

    def print_help(self):
        """도움말 출력"""
        print("\n" + "="*60)
        print("🎮 데모 제스처 제어 - 키보드 명령어")
        print("="*60)
        print("제스처 시뮬레이션:")
        print("  1: 🤏 핀치 (그리퍼 토글)")
        print("  2: ✋ 오픈 핸드 (홈 포지션)")
        print("  3: ✊ 주먹 (비상 정지)")
        print("  4: 🖐️ 앞으로 (앞으로 이동)")
        print("  5: 🖐️ 뒤로 (뒤로 이동)")
        print("  6: 🖐️ 왼쪽 (왼쪽으로 이동)")
        print("  7: 🖐️ 오른쪽 (오른쪽으로 이동)")
        print("  8: 🖐️ 위로 (위로 이동)")
        print("  9: 🖐️ 아래로 (아래로 이동)")
        print()
        print("시스템 제어:")
        print("  h: 🏠 홈 포지션")
        print("  s: 🛡️ 안전 위치")
        print("  e: 🚨 비상 정지")
        print("  c: 🔧 그리퍼 토글")
        print("  r: 🔄 로봇 상태 초기화")
        print("  t: 📊 상태 표시")
        print("  q: 🚪 종료")
        print("  ?: 📚 도움말")

    def simulate_gesture(self, gesture_type: GestureType) -> bool:
        """제스처 시뮬레이션"""
        print(f"\n🎯 제스처 시뮬레이션: {gesture_type.value}")

        self.current_gesture = gesture_type
        success = self.gesture_mapper.execute_gesture(gesture_type)

        if success:
            print(f"✅ 제스처 '{gesture_type.value}' 실행 성공")
        else:
            print(f"❌ 제스처 '{gesture_type.value}' 실행 실패")

        return success

    def run(self):
        """메인 실행 루프"""
        if not self.initialize_system():
            return

        self.running = True
        self.print_help()
        self.print_status()

        print("\n🎮 데모 모드 시작! 명령어를 입력하세요 (? = 도움말, q = 종료):")

        try:
            while self.running:
                try:
                    command = input("\n> ").strip().lower()

                    if command == 'q':
                        break
                    elif command == '?':
                        self.print_help()
                    elif command == 't':
                        self.print_status()
                    elif command == 'h':
                        print("🏠 홈 포지션으로 이동")
                        self.robot_controller.home_position()
                    elif command == 's':
                        print("🛡️ 안전 위치로 이동")
                        self.robot_controller.safe_position()
                    elif command == 'e':
                        print("🚨 비상 정지!")
                        self.robot_controller.emergency_stop()
                    elif command == 'c':
                        print("🔧 그리퍼 토글")
                        self.robot_controller.control_gripper(not self.robot_controller.gripper_closed)
                    elif command == 'r':
                        print("🔄 로봇 상태 초기화")
                        self.robot_controller.home_position()
                    elif command == '1':
                        self.simulate_gesture(GestureType.PINCH)
                    elif command == '2':
                        self.simulate_gesture(GestureType.OPEN_HAND)
                    elif command == '3':
                        self.simulate_gesture(GestureType.FIST)
                    elif command == '4':
                        self.simulate_gesture(GestureType.PALM_FORWARD)
                    elif command == '5':
                        self.simulate_gesture(GestureType.PALM_BACKWARD)
                    elif command == '6':
                        self.simulate_gesture(GestureType.PALM_LEFT)
                    elif command == '7':
                        self.simulate_gesture(GestureType.PALM_RIGHT)
                    elif command == '8':
                        self.simulate_gesture(GestureType.PALM_UP)
                    elif command == '9':
                        self.simulate_gesture(GestureType.PALM_DOWN)
                    elif command == '':
                        continue
                    else:
                        print(f"❓ 알 수 없는 명령어: '{command}'. '?'를 입력하여 도움말을 확인하세요.")

                except KeyboardInterrupt:
                    break
                except EOFError:
                    break

        except KeyboardInterrupt:
            print("\n⚠️ 사용자 중단")
        finally:
            self.cleanup()

    def cleanup(self):
        """시스템 정리"""
        print("\n🔧 시스템 정리 중...")

        self.running = False

        # 로봇 안전 위치로 이동
        if self.robot_controller.is_connected:
            print("🏠 안전 위치로 이동 중...")
            self.robot_controller.safe_position()

        # 통계 출력
        stats = self.gesture_mapper.get_statistics()
        print(f"\n📊 세션 통계:")
        print(f"  총 동작 수: {stats.get('total_actions', 0)}")
        print(f"  성공률: {stats.get('success_rate', '0%')}")
        print(f"  평균 실행 시간: {stats.get('average_duration', '0s')}")

        print("✅ 데모 모드 종료 완료")

def main():
    """메인 함수"""
    print("🎮 RoArm-M2-S 데모 제스처 제어 시스템")
    print("카메라 없이 키보드로 제스처를 시뮬레이션합니다.")

    config = DemoConfig(
        robot_ip="192.168.4.1",
        debug_mode=True
    )

    system = DemoGestureControl(config)
    system.run()

if __name__ == "__main__":
    main()