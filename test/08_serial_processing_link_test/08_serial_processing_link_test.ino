/*
  08_serial_processing_link_test
  Serial packet generator for Processing link verification.

  Packet format:
  angle,distance,state,azimuth,elevation
*/

int angleValue = 0;
int angleStep = 4;

void printIntro() {
  Serial.println(F("# START: 08_serial_processing_link_test"));
  Serial.println(F("# PART: Serial + Processing link"));
  Serial.println(F("# OUTPUT: angle,distance,state,azimuth,elevation"));
  Serial.println(F("# PASS CHECK: viewer should connect, parse packets, and update all five values"));
  Serial.println(F("# FAIL SYMPTOM: missing packets, frozen values, wrong field order"));
  Serial.println(F("#"));
}

void setup() {
  Serial.begin(115200);
  delay(250);
  printIntro();
}

void loop() {
  static unsigned long lastPacketTime = 0;
  const unsigned long packetIntervalMs = 100;

  if (millis() - lastPacketTime < packetIntervalMs) {
    return;
  }
  lastPacketTime = millis();

  angleValue += angleStep;
  if (angleValue >= 180 || angleValue <= 0) {
    angleStep = -angleStep;
  }

  int phase = (millis() / 3000UL) % 3;
  int stateValue = phase;

  int distanceValue;
  if (phase == 0) {
    distanceValue = map(angleValue, 0, 180, 350, 220);
  } else if (phase == 1) {
    distanceValue = map(angleValue, 0, 180, 180, 70);
  } else {
    distanceValue = map(angleValue, 0, 180, 95, 35);
  }

  int azimuthValue = angleValue;
  int elevationValue = map(distanceValue, 35, 350, 65, 25);

  Serial.print(angleValue);
  Serial.print(',');
  Serial.print(distanceValue);
  Serial.print(',');
  Serial.print(stateValue);
  Serial.print(',');
  Serial.print(azimuthValue);
  Serial.print(',');
  Serial.println(elevationValue);
}
