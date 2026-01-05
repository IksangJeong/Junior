/*
 * 5-DOF Robot Arm Controller
 *
 * Hardware: Arduino + MG996R Servos x5 + Gripper
 * Communication: Serial (115200 baud)
 * Command Format: J1:90,J2:45,J3:60,J4:90,J5:90,G:1
 */

#include <Servo.h>

// ============ PIN CONFIGURATION ============
#define SERVO_J1  2   // Base (좌우 회전)
#define SERVO_J2  4   // Joint 2 (앞뒤 기울임)
#define SERVO_J3  6   // Joint 3 (앞뒤 기울임)
#define SERVO_J4  8   // Joint 4 (앞뒤 기울임)
#define SERVO_J5  10  // Wrist (손목 회전)
#define SERVO_GRIP 12 // Gripper (집게)

// ============ SERVO LIMITS ============
#define SERVO_MIN 0
#define SERVO_MAX 180

// ============ SMOOTHING CONFIG ============
#define SMOOTH_DELAY 15    // ms between smooth steps
#define SMOOTH_STEP 2      // degrees per step

// ============ GLOBAL VARIABLES ============
#define NUM_SERVOS 6  // 5 joints + 1 gripper

Servo servos[NUM_SERVOS];
int servoPins[NUM_SERVOS] = {SERVO_J1, SERVO_J2, SERVO_J3, SERVO_J4, SERVO_J5, SERVO_GRIP};

// Current and target positions
int currentPos[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
int targetPos[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Initial home position
int homePos[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Serial buffer
String inputBuffer = "";
bool commandComplete = false;

// ============ SETUP ============
void setup() {
  Serial.begin(115200);
  Serial.println("5-DOF Robot Arm Controller Ready");

  // Attach all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(homePos[i]);
    currentPos[i] = homePos[i];
    targetPos[i] = homePos[i];
  }

  delay(1000);
  Serial.println("OK");
}

// ============ MAIN LOOP ============
void loop() {
  // Read serial commands
  readSerialCommand();

  // Process complete command
  if (commandComplete) {
    processCommand(inputBuffer);
    inputBuffer = "";
    commandComplete = false;
  }

  // Smooth movement update
  updateServos();
}

// ============ SERIAL READING ============
void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        commandComplete = true;
      }
    } else {
      inputBuffer += c;
    }
  }
}

// ============ COMMAND PROCESSING ============
void processCommand(String cmd) {
  cmd.trim();

  // HOME command
  if (cmd == "HOME") {
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetPos[i] = homePos[i];
    }
    Serial.println("HOME:OK");
    return;
  }

  // STOP command
  if (cmd == "STOP") {
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetPos[i] = currentPos[i];
    }
    Serial.println("STOP:OK");
    return;
  }

  // STATUS command
  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  // Parse joint commands: J1:90,J2:45,J3:60,J4:90,J5:90,G:1
  parseJointCommand(cmd);
}

void parseJointCommand(String cmd) {
  int newTargets[NUM_SERVOS];
  bool updated[NUM_SERVOS] = {false, false, false, false, false, false};

  // Copy current targets
  for (int i = 0; i < NUM_SERVOS; i++) {
    newTargets[i] = targetPos[i];
  }

  // Split by comma
  int startIdx = 0;
  while (startIdx < (int)cmd.length()) {
    int commaIdx = cmd.indexOf(',', startIdx);
    if (commaIdx == -1) commaIdx = cmd.length();

    String token = cmd.substring(startIdx, commaIdx);
    token.trim();

    // Parse token (e.g., "J1:90" or "G:1")
    int colonIdx = token.indexOf(':');
    if (colonIdx > 0) {
      String key = token.substring(0, colonIdx);
      int value = token.substring(colonIdx + 1).toInt();

      // Clamp value
      value = constrain(value, SERVO_MIN, SERVO_MAX);

      // Map key to servo index
      int servoIdx = -1;
      if (key == "J1") servoIdx = 0;
      else if (key == "J2") servoIdx = 1;
      else if (key == "J3") servoIdx = 2;
      else if (key == "J4") servoIdx = 3;
      else if (key == "J5") servoIdx = 4;
      else if (key == "G" || key == "GRIP") servoIdx = 5;

      if (servoIdx >= 0 && servoIdx < NUM_SERVOS) {
        newTargets[servoIdx] = value;
        updated[servoIdx] = true;
      }
    }

    startIdx = commaIdx + 1;
  }

  // Apply new targets
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (updated[i]) {
      targetPos[i] = newTargets[i];
    }
  }

  Serial.println("CMD:OK");
}

// ============ SMOOTH SERVO UPDATE ============
void updateServos() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate < SMOOTH_DELAY) {
    return;
  }
  lastUpdate = now;

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (currentPos[i] != targetPos[i]) {
      // Calculate step
      int diff = targetPos[i] - currentPos[i];
      int step = constrain(diff, -SMOOTH_STEP, SMOOTH_STEP);

      currentPos[i] += step;
      servos[i].write(currentPos[i]);
    }
  }
}

// ============ STATUS REPORTING ============
void printStatus() {
  Serial.print("POS:");
  for (int i = 0; i < 5; i++) {
    Serial.print("J");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(currentPos[i]);
    Serial.print(",");
  }
  Serial.print("G:");
  Serial.println(currentPos[5]);
}
