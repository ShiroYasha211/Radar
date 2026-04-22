/*
  07_dc_motor_l298n_test
  Isolated acceptance test for L298N direction control.
*/

#define MOTOR_IN1 4
#define MOTOR_IN2 2

void printIntro() {
  Serial.println(F("START: 07_dc_motor_l298n_test"));
  Serial.println(F("PART: DC Motor + L298N"));
  Serial.println(F("PINS: IN1=D4, IN2=D2"));
  Serial.println(F("POWER: use external motor supply with shared GND"));
  Serial.println(F("PASS CHECK: clear forward direction, full stop, clear reverse direction"));
  Serial.println(F("FAIL SYMPTOM: no spin, unstable speed, board reset, driver overheating"));
  Serial.println();
}

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void motorForward() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

void motorReverse() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorStop();

  delay(250);
  printIntro();
}

void loop() {
  Serial.println(F("RUNNING: forward"));
  motorForward();
  delay(800);

  Serial.println(F("RUNNING: stop"));
  motorStop();
  delay(1200);

  Serial.println(F("RUNNING: reverse"));
  motorReverse();
  delay(800);

  Serial.println(F("RUNNING: stop"));
  motorStop();
  Serial.println(F("PASS CHECK: note the real forward direction for final launch simulation"));
  Serial.println(F("CYCLE COMPLETE"));
  Serial.println();
  delay(1800);
}
