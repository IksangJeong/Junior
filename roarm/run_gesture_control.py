#!/usr/bin/env python3
"""
Quick Start Launcher for RoArm-M2-S Gesture Control
빠른 시작을 위한 실행 스크립트
"""

import sys
import os

def check_dependencies():
    """의존성 확인"""
    required_modules = ['cv2', 'mediapipe', 'numpy', 'requests']
    missing_modules = []

    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing_modules.append(module)

    if missing_modules:
        print("❌ 다음 모듈이 누락되었습니다:")
        for module in missing_modules:
            print(f"  - {module}")
        print("\n설치 명령:")
        print("  pip install -r requirements.txt")
        return False

    return True

def main():
    print("🤖 RoArm-M2-S 제스처 제어 시스템")
    print("=" * 40)

    # 의존성 확인
    if not check_dependencies():
        return

    # 메인 프로그램 실행
    try:
        from main_gesture_control import main as gesture_main
        gesture_main()
    except KeyboardInterrupt:
        print("\n👋 시스템 종료")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        print("\n문제해결:")
        print("1. setup.py를 실행해보세요")
        print("2. 모든 의존성이 설치되었는지 확인하세요")
        print("3. 카메라와 로봇팔 연결을 확인하세요")

if __name__ == "__main__":
    main()