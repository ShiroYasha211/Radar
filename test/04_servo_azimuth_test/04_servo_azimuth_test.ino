/*
  04_servo_azimuth_test
  Isolated acceptance test for the azimuth servo on D9.
*/

#include <Servo.h>

#define SERVO_PIN 9
#define ZERO_AZIMUTH_ANGLE 90
#define TEST_MIN_ANGLE 0
#define TEST_MAX_ANGLE 180
#define STEP_DELAY_MS 15
#define HOLD_DELAY_MS 1200

Servo azimuthServo;
int currentAngle = ZERO_AZIMUTH_ANGLE;

void printIntro() {
  Serial.println(F("START: 04_servo_azimuth_test"));
  Serial.println(F("PART: Servo Azimuth"));
  Serial.println(F("PINS: SERVO_PIN=D9"));
  Serial.println(F("ZERO: ZERO_AZIMUTH_ANGLE is the startup home position"));
  Serial.println(F("POWER: use external 5V/6V supply with shared GND"));
  Serial.println(F("PASS CHECK: smooth motion, correct center, no strong jitter at rest"));
  Serial.println(F("FAIL SYMPTOM: chatter, brownout reset, stalling, hitting end stops"));
  Serial.println(F("NOTE: if the real zero is not correct, change ZERO_AZIMUTH_ANGLE and upload again"));
  Serial.println();
}

void moveSmooth(int targetAngle, int stepDelayMs) {
  int step = (targetAngle >= currentAngle) ? 1 : -1;
  while (currentAngle != targetAngle) {
    currentAngle += step;
    azimuthServo.write(currentAngle);
    delay(stepDelayMs);
  }
}

void setup() {
  Serial.begin(115200);

  azimuthServo.attach(SERVO_PIN);
  azimuthServo.write(ZERO_AZIMUTH_ANGLE);
  currentAngle = ZERO_AZIMUTH_ANGLE;

  delay(750);
  printIntro();
}

void loop() {
  Serial.println(F("RUNNING: return to zero/home position"));
  moveSmooth(ZERO_AZIMUTH_ANGLE, STEP_DELAY_MS);
  delay(HOLD_DELAY_MS);

  Serial.println(F("PASS CHECK: verify that ZERO_AZIMUTH_ANGLE matches the real forward direction"));
  delay(1000);

  Serial.println(F("RUNNING: sweep zero -> min"));
  moveSmooth(TEST_MIN_ANGLE, STEP_DELAY_MS);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: sweep min -> max"));
  moveSmooth(TEST_MAX_ANGLE, STEP_DELAY_MS);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: sweep max -> zero/home"));
  moveSmooth(ZERO_AZIMUTH_ANGLE, STEP_DELAY_MS);
  Serial.println(F("PASS CHECK: the servo must always return to the exact same zero point"));
  Serial.println(F("CYCLE COMPLETE"));
  Serial.println();
  delay(2000);
}
