/*
  03_ultrasonic_hcsr04_test
  Isolated acceptance test for HC-SR04.
*/

#define TRIG_PIN 8
#define ECHO_PIN A4

const unsigned long ECHO_TIMEOUT_US = 30000UL;

void printIntro() {
  Serial.println(F("START: 03_ultrasonic_hcsr04_test"));
  Serial.println(F("PART: HC-SR04 ultrasonic sensor"));
  Serial.println(F("PINS: TRIG=D8, ECHO=A4"));
  Serial.println(F("POWER: USB is enough for this sensor-only test"));
  Serial.println(F("PASS CHECK: real distance changes with target movement, NO_ECHO is separated from valid range"));
  Serial.println(F("FAIL SYMPTOM: fixed values, noisy jumps, always 0, always timeout"));
  Serial.println();
}

long readEchoDuration() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  delay(250);
  printIntro();
}

void loop() {
  long duration = readEchoDuration();

  if (duration == 0) {
    Serial.println(F("RUNNING: NO_ECHO (timeout)"));
  } else {
    float distanceCm = duration * 0.0343f * 0.5f;

    if (distanceCm > 400.0f) {
      Serial.print(F("RUNNING: OUT_OF_RANGE > 400 cm, measured="));
      Serial.print(distanceCm, 1);
      Serial.println(F(" cm"));
    } else {
      Serial.print(F("RUNNING: DISTANCE="));
      Serial.print(distanceCm, 1);
      Serial.println(F(" cm"));
    }
  }

  Serial.println(F("PASS CHECK: move an object closer/farther and verify a logical response"));
  delay(250);
}
