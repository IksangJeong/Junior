/*
 * Hand Tracking Servo Controller
 *
 * Python에서 전송된 손가락 각도 데이터를 받아
 * 5개의 서보모터를 제어하는 Arduino 코드
 *
 * 서보 연결:
 * - 엄지(Thumb): D2
 * - 검지(Index): D6
 * - 중지(Middle): D9
 * - 약지(Ring): D10
 * - 소지(Pinky): D12
 */

#include <Servo.h>

// 서보 모터 객체 생성
Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// 서보 핀 정의 (베이스 코드와 동일)
const int THUMB_PIN = 2;
const int INDEX_PIN = 6;
const int MIDDLE_PIN = 9;
const int RING_PIN = 10;
const int PINKY_PIN = 12;

// 각도 값 저장 변수 (초기값 165도 - 손 편 상태)
int thumbAngle = 165;
int indexAngle = 165;
int middleAngle = 165;
int ringAngle = 165;
int pinkyAngle = 165;

// 현재 서보 위치 (부드러운 움직임용)
float currentThumb = 165;
float currentIndex = 165;
float currentMiddle = 165;
float currentRing = 165;
float currentPinky = 165;

// 부드러운 움직임을 위한 스무딩 팩터 (0.1 ~ 1.0)
const float SMOOTHING_FACTOR = 0.2;

// 시리얼 통신 버퍼
String inputString = "";
boolean stringComplete = false;

// 디버그 LED (내장 LED 사용)
const int LED_PIN = 13;

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(9600);
  Serial.println("Hand Tracking Servo Controller Ready");

  // 서보 모터 연결
  thumbServo.attach(THUMB_PIN);
  indexServo.attach(INDEX_PIN);
  middleServo.attach(MIDDLE_PIN);
  ringServo.attach(RING_PIN);
  pinkyServo.attach(PINKY_PIN);

  // 초기 위치로 이동
  resetPosition();

  // LED 설정
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 시작 신호
  Serial.println("Waiting for hand tracking data...");
  Serial.println("Format: Thumb,Index,Middle,Ring,Pinky");
}

void loop() {
  // 시리얼 데이터 처리
  if (stringComplete) {
    processSerialData();
    inputString = "";
    stringComplete = false;
  }

  // 부드러운 서보 움직임 업데이트
  updateServoPositions();

  // 짧은 지연
  delay(10);
}

// 시리얼 이벤트 (데이터 수신)
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      stringComplete = true;
      digitalWrite(LED_PIN, HIGH);  // 데이터 수신 표시
    } else {
      inputString += inChar;
    }
  }
}

// 시리얼 데이터 파싱 및 처리
void processSerialData() {
  // 데이터 형식: "T,I,M,R,P" (각 값은 0-180)
  int commaIndex = 0;
  int lastCommaIndex = 0;
  int angleIndex = 0;
  int angles[5] = {165, 165, 165, 165, 165};  // 기본값 (손 편 상태)

  // 콤마로 구분된 값들을 파싱
  for (int i = 0; i < inputString.length(); i++) {
    if (inputString.charAt(i) == ',') {
      String angleStr = inputString.substring(lastCommaIndex, i);
      angles[angleIndex] = angleStr.toInt();
      angleIndex++;
      lastCommaIndex = i + 1;

      if (angleIndex >= 4) break;  // 마지막 값 제외
    }
  }

  // 마지막 값 (소지 각도) 파싱
  if (lastCommaIndex < inputString.length()) {
    String angleStr = inputString.substring(lastCommaIndex);
    angles[4] = angleStr.toInt();
  }

  // 각도 값 업데이트 (범위 제한 적용)
  thumbAngle = constrain(angles[0], 0, 180);
  indexAngle = constrain(angles[1], 0, 180);
  middleAngle = constrain(angles[2], 0, 180);
  ringAngle = constrain(angles[3], 0, 180);
  pinkyAngle = constrain(angles[4], 0, 180);

  // 디버그 출력
  Serial.print("Received - T:");
  Serial.print(thumbAngle);
  Serial.print(" I:");
  Serial.print(indexAngle);
  Serial.print(" M:");
  Serial.print(middleAngle);
  Serial.print(" R:");
  Serial.print(ringAngle);
  Serial.print(" P:");
  Serial.println(pinkyAngle);

  digitalWrite(LED_PIN, LOW);  // LED 끄기
}

// 부드러운 서보 위치 업데이트
void updateServoPositions() {
  // 각 서보의 현재 위치를 목표 위치로 부드럽게 이동
  currentThumb += (thumbAngle - currentThumb) * SMOOTHING_FACTOR;
  currentIndex += (indexAngle - currentIndex) * SMOOTHING_FACTOR;
  currentMiddle += (middleAngle - currentMiddle) * SMOOTHING_FACTOR;
  currentRing += (ringAngle - currentRing) * SMOOTHING_FACTOR;
  currentPinky += (pinkyAngle - currentPinky) * SMOOTHING_FACTOR;

  // 서보 모터 위치 설정
  thumbServo.write((int)currentThumb);
  indexServo.write((int)currentIndex);
  middleServo.write((int)currentMiddle);
  ringServo.write((int)currentRing);
  pinkyServo.write((int)currentPinky);
}

// 초기 위치로 리셋
void resetPosition() {
  // 모든 서보를 초기 위치(165도 - 손 편 상태)로 설정
  thumbServo.write(165);
  indexServo.write(165);
  middleServo.write(165);
  ringServo.write(165);
  pinkyServo.write(165);

  currentThumb = 165;
  currentIndex = 165;
  currentMiddle = 165;
  currentRing = 165;
  currentPinky = 165;

  Serial.println("Servos reset to open hand position (165 degrees)");
}

// 테스트 모드 (각 서보를 순차적으로 움직임)
void testServos() {
  Serial.println("Testing all servos...");

  // 각 서보를 165도(펴진 상태)에서 0도(접힌 상태)로 움직임
  for (int angle = 165; angle >= 0; angle -= 30) {
    thumbServo.write(angle);
    delay(200);
  }
  thumbServo.write(165);
  delay(500);

  for (int angle = 165; angle >= 0; angle -= 30) {
    indexServo.write(angle);
    delay(200);
  }
  indexServo.write(165);
  delay(500);

  for (int angle = 165; angle >= 0; angle -= 30) {
    middleServo.write(angle);
    delay(200);
  }
  middleServo.write(165);
  delay(500);

  for (int angle = 165; angle >= 0; angle -= 30) {
    ringServo.write(angle);
    delay(200);
  }
  ringServo.write(165);
  delay(500);

  for (int angle = 165; angle >= 0; angle -= 30) {
    pinkyServo.write(angle);
    delay(200);
  }
  pinkyServo.write(165);
  delay(500);

  Serial.println("Servo test complete");
}