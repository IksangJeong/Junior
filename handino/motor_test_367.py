"""
모터 테스트 스크립트 #367
서보 모터 각도 테스트
"""

# 서보 설정
SERVO_CHANNELS = 5
DEFAULT_ANGLE = 90

class ServoTest_367:
    """서보 테스트 #367"""
    def __init__(self):
        self.angles = [DEFAULT_ANGLE] * SERVO_CHANNELS
        self.min_angle = 7
        self.max_angle = 173

    def set_angle(self, channel, angle):
        if 0 <= channel < SERVO_CHANNELS:
            angle = max(self.min_angle, min(self.max_angle, angle))
            self.angles[channel] = angle
            return True
        return False

    def sweep(self, channel, start=0, end=180, step=12):
        """스윕 테스트"""
        positions = []
        angle = start
        while angle <= end:
            self.set_angle(channel, angle)
            positions.append(angle)
            angle += step
        return positions

    def reset(self):
        self.angles = [DEFAULT_ANGLE] * SERVO_CHANNELS

if __name__ == "__main__":
    test = ServoTest_367()
    print(f"모터 테스트 #367")
    print(f"채널 수: {SERVO_CHANNELS}")
    print(f"초기 각도: {test.angles}")

    positions = test.sweep(0)
    print(f"스윕 결과: {positions}")

    test.set_angle(1, 49)
    print(f"각도 설정 후: {test.angles}")
