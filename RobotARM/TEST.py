import serial
import time
import keyboard

class RobotArm:
   def __init__(self, port='COM4', baudrate=57600):
       self.ser = serial.Serial(port, baudrate)
       time.sleep(2)  # 아두이노 리셋 대기
       
       # 현재 각도 저장
       self.current_angles = {
           1: 90,  # 베이스
           2: 90,  # 첫번째 관절
           3: 90,  # 두번째 관절
           4: 90,  # 세번째 관절
           5: 90,  # 그리퍼 방향
           6: 90   # 그리퍼
       }
       
   def move_servo(self, servo_num, angle):
       # 각도 제한
       if servo_num != 6:  # 그리퍼 제외
           angle = max(10, min(170, angle))

       # 1번(베이스) 모터인 경우 데드밴드 적용
       if servo_num == 1:
           current_angle = self.current_angles[servo_num]
           if abs(current_angle - angle) <= 3:  # 3도 이내의 오차는 무시
               return

       command = f"S{servo_num},{angle}\n"
       print(f"Sending command: {command}")  # 디버깅 출력
       self.ser.write(command.encode())
       self.current_angles[servo_num] = angle
       
   def close(self):
       self.ser.close()

def main():
   arm = RobotArm()
   step = 2  # 2도씩 이동
   
   print("제어 방법:")
   print("← → : 베이스 회전")
   print("↑ ↓ : 관절1 제어")
   print("W S : 관절2 제어")
   print("E D : 관절3 제어")
   print("Z X : 그리퍼 방향 제어")
   print("Space : 그리퍼 열기/닫기")
   print("q : 종료")
   
   print(f"Current COM port: {arm.ser.port}")
   print(f"Serial connected: {arm.ser.is_open}")
   
   gripper_open = True
   
   try:
       while True:
           if keyboard.is_pressed('left'):
               new_angle = arm.current_angles[1] + step
               arm.move_servo(1, new_angle)
               print(f"Base: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('right'):
               new_angle = arm.current_angles[1] - step
               arm.move_servo(1, new_angle)
               print(f"Base: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('up'):
               new_angle = arm.current_angles[2] - step
               arm.move_servo(2, new_angle)
               print(f"Joint1: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('down'):
               new_angle = arm.current_angles[2] + step
               arm.move_servo(2, new_angle)
               print(f"Joint1: {new_angle}°")
               time.sleep(0.15)

           if keyboard.is_pressed('w'):
               new_angle = arm.current_angles[3] + step
               arm.move_servo(3, new_angle)
               print(f"Joint2: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('s'):
               new_angle = arm.current_angles[3] - step
               arm.move_servo(3, new_angle)
               print(f"Joint2: {new_angle}°")
               time.sleep(0.15)

           if keyboard.is_pressed('e'):
               new_angle = arm.current_angles[4] + step
               arm.move_servo(4, new_angle)
               print(f"Joint3: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('d'):
               new_angle = arm.current_angles[4] - step
               arm.move_servo(4, new_angle)
               print(f"Joint3: {new_angle}°")
               time.sleep(0.15)

           if keyboard.is_pressed('z'):
               new_angle = arm.current_angles[5] + step
               arm.move_servo(5, new_angle)
               print(f"Gripper Direction: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('x'):
               new_angle = arm.current_angles[5] - step
               arm.move_servo(5, new_angle)
               print(f"Gripper Direction: {new_angle}°")
               time.sleep(0.15)
               
           if keyboard.is_pressed('space'):
               gripper_open = not gripper_open
               angle = 90 if gripper_open else 178
               arm.move_servo(6, angle)
               print(f"Gripper: {'Open' if gripper_open else 'Close'}")
               time.sleep(0.3)
               
           if keyboard.is_pressed('q'):
               break
               
   except KeyboardInterrupt:
       pass
   finally:
       arm.close()
       print("프로그램 종료")

if __name__ == "__main__":
   main()