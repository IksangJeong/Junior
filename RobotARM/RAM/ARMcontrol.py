import serial
import time

class RobotArm:
    def __init__(self, port='COM4', baudrate=57600):
        self.ser = serial.Serial(port, baudrate)
        time.sleep(2)  # 아두이노 리셋 대기
        
    def move_servo(self, servo_num, angle):
        """
        서보모터 제어 함수
        servo_num: 1-6 (모터 번호)
        angle: 각도 (모터 6번 제외하고는 10-170도로 제한됨)
        """
        if not (1 <= servo_num <= 6):
            raise ValueError("서보 번호는 1-6 사이여야 합니다")
            
        # 명령어 전송
        command = f"S{servo_num},{angle}\n"
        self.ser.write(command.encode())
        
        # 응답 대기
        response = self.ser.readline().decode().strip()
        return response
        
    def grip(self, close=True):
        """집게 제어 함수"""
        angle = 180 if close else 70
        return self.move_servo(6, angle)
        
    def close(self):
        """시리얼 포트 닫기"""
        self.ser.close()

# 사용 예시
if __name__ == "__main__":
    arm = RobotArm()  # COM4 포트에 연결
    
    try:
        # 기본 동작 테스트
        print("베이스 회전")
        arm.move_servo(1, 120)  # 베이스를 45도로
        time.sleep(1)
        
        print("첫 번째 관절")
        arm.move_servo(2, 70)  # 첫 번째 관절을 120도로
        time.sleep(1)
        
        print("집게 테스트")
        arm.grip(False)  # 집게 열기
        time.sleep(1)
        arm.grip(True)   # 집게 닫기
        
    except Exception as e:
        print(f"에러 발생: {e}")
        
    finally:
        arm.close()