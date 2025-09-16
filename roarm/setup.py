#!/usr/bin/env python3
"""
RoArm-M2-S Gesture Control Setup Script
설치 및 초기 설정 스크립트
"""

import os
import sys
import subprocess
import json
from pathlib import Path

def check_python_version():
    """Python 버전 확인"""
    if sys.version_info < (3, 8):
        print("❌ Python 3.8 이상이 필요합니다.")
        print(f"현재 버전: {sys.version}")
        return False
    print(f"✅ Python 버전 확인: {sys.version}")
    return True

def install_requirements():
    """필요 라이브러리 설치"""
    print("📦 필요 라이브러리 설치 중...")

    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
        print("✅ 라이브러리 설치 완료")
        return True
    except subprocess.CalledProcessError:
        print("❌ 라이브러리 설치 실패")
        return False

def test_camera():
    """카메라 테스트"""
    print("📷 카메라 테스트 중...")

    try:
        import cv2
        cap = cv2.VideoCapture(0)

        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print("✅ 카메라 테스트 성공")
                cap.release()
                return True
            else:
                print("❌ 카메라에서 프레임을 읽을 수 없습니다.")
        else:
            print("❌ 카메라를 열 수 없습니다.")

        cap.release()
        return False

    except ImportError:
        print("❌ OpenCV가 설치되지 않았습니다.")
        return False

def test_mediapipe():
    """MediaPipe 테스트"""
    print("🤖 MediaPipe 테스트 중...")

    try:
        import mediapipe as mp
        hands = mp.solutions.hands.Hands()
        print("✅ MediaPipe 테스트 성공")
        return True
    except ImportError:
        print("❌ MediaPipe가 설치되지 않았습니다.")
        return False

def create_config_file():
    """기본 설정 파일 생성"""
    print("⚙️ 설정 파일 생성 중...")

    config = {
        "robot": {
            "ip": "192.168.4.1",
            "port": 80,
            "timeout": 5,
            "retry_count": 3
        },
        "camera": {
            "index": 0,
            "width": 640,
            "height": 480,
            "fps": 30
        },
        "gesture": {
            "pinch_threshold": 0.04,
            "confidence_threshold": 0.6,
            "cooldown_time": 1.0,
            "move_step": 15
        },
        "safety": {
            "workspace_limits": {
                "x_min": 50,
                "x_max": 400,
                "y_min": -300,
                "y_max": 300,
                "z_min": 50,
                "z_max": 300
            },
            "emergency_stop_enabled": True,
            "max_speed": 100
        }
    }

    with open("config.json", "w") as f:
        json.dump(config, f, indent=2)

    print("✅ 설정 파일 생성 완료: config.json")

def create_launcher_script():
    """실행 스크립트 생성"""
    print("🚀 실행 스크립트 생성 중...")

    # Windows 배치 파일
    batch_content = """@echo off
echo RoArm-M2-S Gesture Control System
echo ===================================
python main_gesture_control.py
pause
"""

    with open("run_gesture_control.bat", "w") as f:
        f.write(batch_content)

    # Unix/Linux 쉘 스크립트
    shell_content = """#!/bin/bash
echo "RoArm-M2-S Gesture Control System"
echo "=================================="
python3 main_gesture_control.py
"""

    with open("run_gesture_control.sh", "w") as f:
        f.write(shell_content)

    # 실행 권한 부여 (Unix/Linux)
    if os.name != 'nt':
        os.chmod("run_gesture_control.sh", 0o755)

    print("✅ 실행 스크립트 생성 완료")

def main():
    """메인 설정 함수"""
    print("🔧 RoArm-M2-S 제스처 제어 시스템 설정")
    print("=" * 50)

    # 단계별 설정
    steps = [
        ("Python 버전 확인", check_python_version),
        ("라이브러리 설치", install_requirements),
        ("카메라 테스트", test_camera),
        ("MediaPipe 테스트", test_mediapipe),
        ("설정 파일 생성", create_config_file),
        ("실행 스크립트 생성", create_launcher_script)
    ]

    success_count = 0

    for step_name, step_func in steps:
        print(f"\n{step_name}...")
        if step_func():
            success_count += 1
        else:
            print(f"❌ {step_name} 실패")

    print(f"\n📊 설정 완료: {success_count}/{len(steps)} 단계 성공")

    if success_count == len(steps):
        print("\n🎉 모든 설정이 완료되었습니다!")
        print("\n다음 명령으로 시스템을 실행할 수 있습니다:")
        print("  python main_gesture_control.py")
        print("  또는 run_gesture_control.bat (Windows)")
        print("  또는 ./run_gesture_control.sh (Linux/Mac)")

        print("\n📋 시작하기 전에:")
        print("1. RoArm-M2-S 로봇팔 전원을 켜세요")
        print("2. 로봇팔의 Wi-Fi AP에 연결하세요 (기본: 192.168.4.1)")
        print("3. 웹캠이 제대로 연결되어 있는지 확인하세요")
        print("4. 충분한 조명이 있는 환경에서 사용하세요")

    else:
        print("\n⚠️ 일부 설정에 문제가 있습니다. 위의 오류를 확인하고 다시 시도하세요.")

if __name__ == "__main__":
    main()