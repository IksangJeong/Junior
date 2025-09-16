#!/usr/bin/env python3
"""
간단한 카메라 테스트
"""

import cv2
import time

def test_camera():
    print("🔍 카메라 테스트 시작...")

    # 카메라 초기화
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ 카메라 열기 실패")
        return False

    print("✅ 카메라 열기 성공")

    # 카메라 속성 확인
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"📷 카메라 설정: {width:.0f}x{height:.0f} @ {fps:.1f}fps")

    # 몇 프레임 테스트
    for i in range(5):
        ret, frame = cap.read()
        if not ret:
            print(f"❌ 프레임 {i+1} 읽기 실패")
            cap.release()
            return False
        print(f"✅ 프레임 {i+1}: {frame.shape}")
        time.sleep(0.1)

    cap.release()
    print("✅ 카메라 테스트 완료")
    return True

if __name__ == "__main__":
    test_camera()