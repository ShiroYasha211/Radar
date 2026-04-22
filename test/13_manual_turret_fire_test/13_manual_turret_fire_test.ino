/*
  13_manual_turret_fire_test
  Manual combined test for:
  - azimuth servo on D9
  - elevation servo on D5
  - launch DC motor through L298N on D4/D2
*/

#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define AZIMUTH_PIN 9
#define ELEVATION_PIN 5

#define MOTOR_IN1 4
#define MOTOR_IN2 2

#define ZERO_AZIMUTH_ANGLE 60
#define ZERO_ELEVATION_ANGLE 45

#define AZIMUTH_MIN_ANGLE 0
#define AZIMUTH_MAX_ANGLE 130
#define ELEVATION_MIN_ANGLE 20
#define ELEVATION_MAX_ANGLE 90

#define AZIMUTH_STEP_DELAY_MS 15
#define ELEVATION_STEP_DELAY_MS 15
#define STATUS_REPORT_INTERVAL_MS 300

Servo azimuthServo;
Servo elevationServo;

int currentAzimuth = ZERO_AZIMUTH_ANGLE;
int currentElevation = ZERO_ELEVATION_ANGLE;

unsigned long lastStatusMs = 0;

enum FireState {
  FIRE_STOPPED = 0,
  FIRE_FORWARD = 1,
  FIRE_REVERSE = 2
};

FireState fireState = FIRE_STOPPED;

char commandBuffer[40];
byte commandLength = 0;

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  fireState = FIRE_STOPPED;
}

void motorForward() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  fireState = FIRE_FORWARD;
}

void motorReverse() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  fireState = FIRE_REVERSE;
}

const __FlashStringHelper* fireStateLabel() {
  switch (fireState) {
    case FIRE_FORWARD: return F("FORWARD");
    case FIRE_REVERSE: return F("REVERSE");
    default: return F("STOPPED");
  }
}

void reportStatus(const __FlashStringHelper* label) {
  Serial.print(label);
  Serial.print(F(" | AZ="));
  Serial.print(currentAzimuth);
  Serial.print(F(" deg | EL="));
  Serial.print(currentElevation);
  Serial.print(F(" deg | FIRE="));
  Serial.println(fireStateLabel());
}

void printIntro() {
  Serial.println(F("START: 13_manual_turret_fire_test"));
  Serial.println(F("PART: Manual turret + launch mechanism wiring validation"));
  Serial.println(F("PINS: AZIMUTH=D9, ELEVATION=D5, MOTOR_IN1=D4, MOTOR_IN2=D2"));
  Serial.println(F("L298N: use Channel A only for this test"));
  Serial.println(F("L298N WIRING: IN1<-D4, IN2<-D2, OUT1/OUT2->launch motor, GND مشتركة"));
  Serial.println(F("POWER: external supply for servo and motor, never USB only"));
  Serial.println(F("NOTE: keep ENA jumper installed if you only want ON/OFF motor test"));
  Serial.println(F("PASS CHECK: both servos move correctly and launch motor spins on command"));
  Serial.println(F("FAIL SYMPTOM: wrong direction, jitter, no spin, reset, overheating"));
  Serial.println();
}

void printHelp() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  a<number>   -> set azimuth angle, example: a90"));
  Serial.println(F("  e<number>   -> set elevation angle, example: e55"));
  Serial.println(F("  az+         -> azimuth +1 degree"));
  Serial.println(F("  az-         -> azimuth -1 degree"));
  Serial.println(F("  az+5        -> azimuth +5 degrees"));
  Serial.println(F("  az-5        -> azimuth -5 degrees"));
  Serial.println(F("  el+         -> elevation +1 degree"));
  Serial.println(F("  el-         -> elevation -1 degree"));
  Serial.println(F("  el+5        -> elevation +5 degrees"));
  Serial.println(F("  el-5        -> elevation -5 degrees"));
  Serial.println(F("  home        -> return both servos to zero/home"));
  Serial.println(F("  fire        -> motor forward"));
  Serial.println(F("  rev         -> motor reverse"));
  Serial.println(F("  stop        -> motor stop"));
  Serial.println(F("  pulse300    -> forward pulse for 300 ms"));
  Serial.println(F("  pulse600    -> forward pulse for 600 ms"));
  Serial.println(F("  s           -> show current status"));
  Serial.println(F("  ?           -> show help"));
  Serial.println();
}

void moveAzimuthSmooth(int targetAngle) {
  targetAngle = constrain(targetAngle, AZIMUTH_MIN_ANGLE, AZIMUTH_MAX_ANGLE);
  if (targetAngle == currentAzimuth) {
    reportStatus(F("AZ HOLD"));
    return;
  }

  int step = (targetAngle > currentAzimuth) ? 1 : -1;
  while (currentAzimuth != targetAngle) {
    currentAzimuth += step;
    azimuthServo.write(currentAzimuth);
    reportStatus(F("AZ MOVE"));
    delay(AZIMUTH_STEP_DELAY_MS);
  }
}

void moveElevationSmooth(int targetAngle) {
  targetAngle = constrain(targetAngle, ELEVATION_MIN_ANGLE, ELEVATION_MAX_ANGLE);
  if (targetAngle == currentElevation) {
    reportStatus(F("EL HOLD"));
    return;
  }

  int step = (targetAngle > currentElevation) ? 1 : -1;
  while (currentElevation != targetAngle) {
    currentElevation += step;
    elevationServo.write(currentElevation);
    reportStatus(F("EL MOVE"));
    delay(ELEVATION_STEP_DELAY_MS);
  }
}

void moveHome() {
  moveAzimuthSmooth(ZERO_AZIMUTH_ANGLE);
  moveElevationSmooth(ZERO_ELEVATION_ANGLE);
  reportStatus(F("HOME"));
}

void firePulse(unsigned long pulseMs) {
  motorForward();
  reportStatus(F("FIRE PULSE"));
  delay(pulseMs);
  motorStop();
  reportStatus(F("FIRE STOP"));
}

bool isWhitespaceChar(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

char* trimCommand(char* text) {
  while (*text && isWhitespaceChar(*text)) {
    text++;
  }

  int len = strlen(text);
  while (len > 0 && isWhitespaceChar(text[len - 1])) {
    text[len - 1] = '\0';
    len--;
  }

  return text;
}

void normalizeCommand(char* text) {
  for (int i = 0; text[i] != '\0'; i++) {
    text[i] = (char)tolower((unsigned char)text[i]);
  }
}

void handleCommand(char* rawCommand) {
  char* command = trimCommand(rawCommand);
  normalizeCommand(command);
  if (command[0] == '\0') return;

  if (strcmp(command, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(command, "s") == 0 || strcmp(command, "status") == 0) {
    reportStatus(F("STATUS"));
    return;
  }

  if (strcmp(command, "home") == 0 || strcmp(command, "h") == 0) {
    moveHome();
    return;
  }

  if (strcmp(command, "fire") == 0) {
    motorForward();
    reportStatus(F("FIRE ON"));
    return;
  }

  if (strcmp(command, "rev") == 0) {
    motorReverse();
    reportStatus(F("FIRE REV"));
    return;
  }

  if (strcmp(command, "stop") == 0) {
    motorStop();
    reportStatus(F("FIRE OFF"));
    return;
  }

  if (strcmp(command, "pulse300") == 0) {
    firePulse(300);
    return;
  }

  if (strcmp(command, "pulse600") == 0) {
    firePulse(600);
    return;
  }

  if (strcmp(command, "az+") == 0) {
    moveAzimuthSmooth(currentAzimuth + 1);
    return;
  }

  if (strcmp(command, "az-") == 0) {
    moveAzimuthSmooth(currentAzimuth - 1);
    return;
  }

  if (strcmp(command, "el+") == 0) {
    moveElevationSmooth(currentElevation + 1);
    return;
  }

  if (strcmp(command, "el-") == 0) {
    moveElevationSmooth(currentElevation - 1);
    return;
  }

  if (strncmp(command, "az", 2) == 0) {
    moveAzimuthSmooth(currentAzimuth + atoi(command + 2));
    return;
  }

  if (strncmp(command, "el", 2) == 0) {
    moveElevationSmooth(currentElevation + atoi(command + 2));
    return;
  }

  if (command[0] == 'a') {
    moveAzimuthSmooth(atoi(command + 1));
    reportStatus(F("AZ SET"));
    return;
  }

  if (command[0] == 'e') {
    moveElevationSmooth(atoi(command + 1));
    reportStatus(F("EL SET"));
    return;
  }

  Serial.println(F("UNKNOWN COMMAND"));
  printHelp();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      commandBuffer[0] = '\0';
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorStop();

  azimuthServo.attach(AZIMUTH_PIN);
  elevationServo.attach(ELEVATION_PIN);
  azimuthServo.write(ZERO_AZIMUTH_ANGLE);
  elevationServo.write(ZERO_ELEVATION_ANGLE);

  currentAzimuth = ZERO_AZIMUTH_ANGLE;
  currentElevation = ZERO_ELEVATION_ANGLE;

  delay(900);
  printIntro();
  printHelp();
  reportStatus(F("HOME"));
}

void loop() {
  readSerialCommands();

  unsigned long nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_REPORT_INTERVAL_MS) {
    lastStatusMs = nowMs;
    reportStatus(F("LIVE"));
  }
}
