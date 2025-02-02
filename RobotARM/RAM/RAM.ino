#include <Servo.h>

Servo servo1, servo2, servo3, servo4, servo5, servo6;

// 각도 제한 함수
int constrainAngle(int angle) {
   return constrain(angle, 10, 170);
}

// 현재 각도 저장 변수
int currentAngles[6] = {90, 90, 90, 90, 90, 90};

void setup() {
   Serial.begin(57600);
   
   // 서보모터 연결
   servo1.attach(2);   // 베이스
   servo2.attach(4);   // 첫번째 관절
   servo3.attach(6);   // 두번째 관절
   servo4.attach(8);   // 세번째 관절
   servo5.attach(10);  // 집게 방향
   servo6.attach(12);  // 집게 개폐
   
   // 초기 위치 설정 (순차적으로)
   moveServo(1, 90);
   delay(500);
   moveServo(2, 90);
   delay(500);
   moveServo(3, 90);
   delay(500);
   moveServo(4, 90);
   delay(500);
   moveServo(5, 90);
   delay(500);
   moveServo(6, 90);
}

void moveServo(int servoNum, int angle) {
   // 집게(6번)를 제외하고 각도 제한
   if(servoNum != 6) {
       angle = constrainAngle(angle);
   }
   
   // 베이스 모터(1번)에 대해서만 데드밴드 적용
   if(servoNum == 1) {
       int currentAngle = currentAngles[servoNum - 1];
       if(abs(currentAngle - angle) <= 3) {  // 3도 이내의 오차는 무시
           return;
       }
   }
   
   switch(servoNum) {
       case 1: servo1.write(angle); break;
       case 2: servo2.write(angle); break;
       case 3: servo3.write(angle); break;
       case 4: servo4.write(angle); break;
       case 5: servo5.write(angle); break;
       case 6: servo6.write(angle); break;
   }
   
   currentAngles[servoNum - 1] = angle;
}

void loop() {
   if (Serial.available() > 0) {
       String data = Serial.readStringUntil('\n');
       
       if(data.startsWith("S")) {
           int servoNum = data.substring(1,2).toInt();
           int angle = data.substring(3).toInt();
           moveServo(servoNum, angle);
       }
   }
}