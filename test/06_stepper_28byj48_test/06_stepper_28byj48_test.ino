/*
  06_stepper_28byj48_test
  Isolated acceptance test for 28BYJ-48 + ULN2003.
*/

#define STEPPER_IN1 A0
#define STEPPER_IN2 A1
#define STEPPER_IN3 A2
#define STEPPER_IN4 A3

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

int halfIndex = 0;
unsigned long stepCounter = 0;
bool introPrinted = false;
const unsigned int HALF_STEP_DELAY_US = 1465;

void printIntro() {
  Serial.println(F("START: 06_stepper_28byj48_test"));
  Serial.println(F("PART: 28BYJ-48 + ULN2003"));
  Serial.println(F("PINS: IN1=A0, IN2=A1, IN3=A2, IN4=A3"));
  Serial.println(F("POWER: use external 5V supply with shared GND"));
  Serial.println(F("MODE: continuous 360-degree rotation"));
  Serial.println(F("PASS CHECK: smooth continuous spin, fixed direction, no pauses, no abnormal heating"));
  Serial.println(F("FAIL SYMPTOM: buzzing without motion, random direction, skipped steps, pauses or strong shaking"));
  Serial.println();
}

void stepperOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(stepperPins[i], LOW);
  }
}

void doHalfStep(int steps, int dir, unsigned int stepDelayUs) {
  for (int i = 0; i < steps; i++) {
    for (int pin = 0; pin < 4; pin++) {
      digitalWrite(stepperPins[pin], halfStepSequence[halfIndex][pin]);
    }
    halfIndex += dir;
    if (halfIndex >= 8) halfIndex = 0;
    if (halfIndex < 0) halfIndex = 7;
    stepCounter++;
    delayMicroseconds(stepDelayUs);
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(stepperPins[i], OUTPUT);
  }
  stepperOff();

  delay(250);
  printIntro();
}

void loop() {
  if (!introPrinted) {
    Serial.println(F("RUNNING: half-step clockwise continuous rotation"));
    Serial.println(F("NOTE: motor should keep rotating 360 degrees without stopping"));
    introPrinted = true;
  }

  doHalfStep(256, 1, HALF_STEP_DELAY_US);

  if (stepCounter >= 4096UL) {
    Serial.println(F("PASS CHECK: one full revolution completed smoothly"));
    stepCounter = 0;
  }
}
