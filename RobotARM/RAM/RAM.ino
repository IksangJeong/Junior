#include <Servo.h>


//모든 서보 모터 10~170 사이에서만 이동 가능. 
// 10도에서 170도 사이에서만 이동 가능하도록 제한
// 1번 서보 모터 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

void setup() {
  // 서보모터 연결
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  servo5.attach(8);
  servo6.attach(9);
  
  // 모든 서보 한 번에 80도로 이동
  servo1.write(80);
  servo2.write(80);
  servo3.write(80);
  servo4.write(80);
  servo5.write(80);
  servo6.write(80);
}

void loop() {
  // 추가 동작 없음
}