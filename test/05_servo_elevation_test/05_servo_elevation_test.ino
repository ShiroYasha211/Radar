/*
  05_servo_elevation_test
  Isolated acceptance test for the elevation servo on D5.
*/

#include <Servo.h>

#define SERVO_PIN 5
#define SAFE_MIN_ANGLE 20
#define ZERO_ELEVATION_ANGLE 45
#define SAFE_MAX_ANGLE 70
#define STEP_DELAY_MS 18
#define HOLD_DELAY_MS 1200

Servo elevationServo;
int currentAngle = ZERO_ELEVATION_ANGLE;

void printIntro() {
  Serial.println(F("START: 05_servo_elevation_test"));
  Serial.println(F("PART: Servo Elevation"));
  Serial.println(F("PINS: SERVO_PIN=D5"));
  Serial.println(F("ZERO: ZERO_ELEVATION_ANGLE is the startup home position"));
  Serial.println(F("POWER: use external 5V/6V supply with shared GND"));
  Serial.println(F("SAFE RANGE: 20 deg to 70 deg"));
  Serial.println(F("PASS CHECK: no collision, no heavy buzz, stable hold inside safe range"));
  Serial.println(F("FAIL SYMPTOM: mechanical hit, load buzz, linkage strain, brownout reset"));
  Serial.println(F("NOTE: if the real zero is not correct, change ZERO_ELEVATION_ANGLE and upload again"));
  Serial.println();
}

void moveSmooth(int targetAngle, int stepDelayMs) {
  int step = (targetAngle >= currentAngle) ? 1 : -1;
  while (currentAngle != targetAngle) {
    currentAngle += step;
    elevationServo.write(currentAngle);
    delay(stepDelayMs);
  }
}

void setup() {
  Serial.begin(115200);

  elevationServo.attach(SERVO_PIN);
  elevationServo.write(ZERO_ELEVATION_ANGLE);
  currentAngle = ZERO_ELEVATION_ANGLE;

  delay(750);
  printIntro();
}

void loop() {
  Serial.println(F("RUNNING: return to zero/home position"));
  moveSmooth(ZERO_ELEVATION_ANGLE, STEP_DELAY_MS);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: sweep zero -> min"));
  moveSmooth(SAFE_MIN_ANGLE, STEP_DELAY_MS);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: sweep min -> max"));
  moveSmooth(SAFE_MAX_ANGLE, STEP_DELAY_MS);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: sweep max -> zero/home"));
  moveSmooth(ZERO_ELEVATION_ANGLE, STEP_DELAY_MS);
  Serial.println(F("PASS CHECK: confirm the safe range and verify repeatable return to zero"));
  Serial.println(F("CYCLE COMPLETE"));
  Serial.println();
  delay(2000);
}
