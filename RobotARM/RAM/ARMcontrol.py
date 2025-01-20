import serial
import time

try:
    # 시리얼 연결
    ser = serial.Serial('COM4', 9600, timeout=1)
    print("연결 성공!")
    time.sleep(2)  # 아두이노 리셋 대기
    
    print("서보모터 번호:")
    print("1: 베이스 회전 (2번 핀)")
    print("2: 첫 번째 관절 (3번 핀)")
    print("3: 두 번째 관절 (4번 핀)")
    print("4: 세 번째 관절 (5번 핀)")
    print("5: 집게 회전 (8번 핀)")
    print("6: 집게 개폐 (9번 핀)")
    
    while True:
        motor = input("\n모터 번호 선택 (1-6, q로 종료): ")
        if motor.lower() == 'q':
            break
            
        angle = input("각도 입력 (0-180): ")
        
        # 모터번호,각도 형식으로 전송
        command = f"M{motor}A{angle}\n"
        ser.write(command.encode())
        print(f"전송: {motor}번 모터, {angle}도")
        
        # 아두이노 응답 읽기
        time.sleep(0.1)
        while ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"아두이노 응답: {response}")
            
except Exception as e:
    print(f"오류 발생: {e}")
    
finally:
    if 'ser' in locals():
        ser.close()
        print("포트 닫힘")