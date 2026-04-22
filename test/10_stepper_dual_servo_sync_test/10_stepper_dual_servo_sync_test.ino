/*
  10_stepper_dual_servo_sync_test
  Synchronized motion test for:
  - 28BYJ-48 + ULN2003 stepper
  - Azimuth servo
  - Elevation servo
*/

#include <Servo.h>

#define STEPPER_IN1 A0
#define STEPPER_IN2 A1
#define STEPPER_IN3 A2
#define STEPPER_IN4 A3

#define AZIMUTH_PIN 9
#define ELEVATION_PIN 5

#define ZERO_AZIMUTH_ANGLE 60
#define ZERO_ELEVATION_ANGLE 45

#define AZIMUTH_MIN_ANGLE 0
#define AZIMUTH_MAX_ANGLE 130
#define ELEVATION_MIN_ANGLE 20
#define ELEVATION_MAX_ANGLE 90

const int stepperPins[4] = {STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4};
const int halfStepSequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

const unsigned long STEPPER_INTERVAL_US = 1800;
const unsigned long SERVO_INTERVAL_MS = 20;
const unsigned long STATUS_INTERVAL_MS = 500;

Servo azimuthServo;
Servo elevationServo;

int stepIndex = 0;
long halfStepPosition = 0;
int stepperAngle = 0;

int currentAzimuth = ZERO_AZIMUTH_ANGLE;
int currentElevation = ZERO_ELEVATION_ANGLE;
int targetAzimuth = AZIMUTH_MAX_ANGLE;
int targetElevation = ELEVATION_MAX_ANGLE;

bool azimuthForward = true;
bool elevationForward = true;

unsigned long lastStepperUs = 0;
unsigned long lastServoMs = 0;
unsigned long lastStatusMs = 0;

void printIntro() {
  Serial.println(F("START: 10_stepper_dual_servo_sync_test"));
  Serial.println(F("PART: Stepper + Azimuth Servo + Elevation Servo"));
  Serial.println(F("PINS: Stepper=A0,A1,A2,A3 | Azimuth=D9 | Elevation=D5"));
  Serial.println(F("POWER: use external power for motors with shared GND"));
  Serial.println(F("MODE: all three motors move at the same time"));
  Serial.println(F("PASS CHECK: smooth synchronized motion, no resets, no hard vibration"));
  Serial.println(F("FAIL SYMPTOM: brownout, servo chatter, skipped steps, strong shaking"));
  Serial.println();
}

int wrapAngle360(int angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

void stepperOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(stepperPins[i], LOW);
  }
}

void doHalfStepForward() {
  stepIndex++;
  if (stepIndex >= 8) stepIndex = 0;

  for (int i = 0; i < 4; i++) {
    digitalWrite(stepperPins[i], halfStepSequence[stepIndex][i]);
  }

  halfStepPosition++;
  if (halfStepPosition >= 4096L) halfStepPosition = 0;
  stepperAngle = wrapAngle360((int)((halfStepPosition * 360L) / 4096L));
}

void moveServosOneStep() {
  if (currentAzimuth < targetAzimuth) {
    currentAzimuth++;
    azimuthServo.write(currentAzimuth);
  } else if (currentAzimuth > targetAzimuth) {
    currentAzimuth--;
    azimuthServo.write(currentAzimuth);
  } else {
    azimuthForward = !azimuthForward;
    targetAzimuth = azimuthForward ? AZIMUTH_MAX_ANGLE : AZIMUTH_MIN_ANGLE;
  }

  if (currentElevation < targetElevation) {
    currentElevation++;
    elevationServo.write(currentElevation);
  } else if (currentElevation > targetElevation) {
    currentElevation--;
    elevationServo.write(currentElevation);
  } else {
    elevationForward = !elevationForward;
    targetElevation = elevationForward ? ELEVATION_MAX_ANGLE : ELEVATION_MIN_ANGLE;
  }
}

void printStatus() {
  Serial.print(F("SYNC | STEP="));
  Serial.print(stepperAngle);
  Serial.print(F(" deg | AZ="));
  Serial.print(currentAzimuth);
  Serial.print(F(" deg | EL="));
  Serial.print(currentElevation);
  Serial.println(F(" deg"));
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(stepperPins[i], OUTPUT);
  }

  azimuthServo.attach(AZIMUTH_PIN);
  elevationServo.attach(ELEVATION_PIN);

  azimuthServo.write(currentAzimuth);
  elevationServo.write(currentElevation);
  stepperOff();

  delay(800);
  printIntro();
  printStatus();
}

void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastStepperUs >= STEPPER_INTERVAL_US) {
    lastStepperUs = nowUs;
    doHalfStepForward();
  }

  unsigned long nowMs = millis();
  if (nowMs - lastServoMs >= SERVO_INTERVAL_MS) {
    lastServoMs = nowMs;
    moveServosOneStep();
  }

  if (nowMs - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = nowMs;
    printStatus();
  }
}
