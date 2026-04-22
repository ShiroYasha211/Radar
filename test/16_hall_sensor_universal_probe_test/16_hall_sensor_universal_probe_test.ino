/*
  16_hall_sensor_universal_probe_test
  Universal probe test for unknown Hall-effect sensors.
  Works with:
  - Digital Hall switches like A3144 (with external 10k pull-up)
  - Analog Hall sensors like SS49E/49E
*/

#define HALL_PIN A0
#define SAMPLE_COUNT 12
#define REPORT_INTERVAL_MS 200

unsigned long lastReportMs = 0;
int baselineValue = -1;

int readFilteredAnalog() {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(HALL_PIN);
    delay(2);
  }
  return (int)(sum / SAMPLE_COUNT);
}

void printIntro() {
  Serial.println(F("START: 16_hall_sensor_universal_probe_test"));
  Serial.println(F("PIN: A0"));
  Serial.println(F("GOAL: identify whether the sensor is digital or analog and whether it reacts to a magnet"));
  Serial.println();
  Serial.println(F("WIRING:"));
  Serial.println(F("  Sensor VCC -> 5V"));
  Serial.println(F("  Sensor GND -> GND"));
  Serial.println(F("  Sensor OUT -> A0"));
  Serial.println(F("  10k resistor between OUT and 5V"));
  Serial.println();
  Serial.println(F("HOW TO TEST:"));
  Serial.println(F("  1. Power the Arduino and open Serial Monitor at 115200"));
  Serial.println(F("  2. Watch the VALUE line with no magnet"));
  Serial.println(F("  3. Bring a strong magnet very close to the sensor"));
  Serial.println(F("  4. Flip the magnet and try the other face too"));
  Serial.println(F("  5. If there is still no change, the pinout or sensor type is likely wrong"));
  Serial.println();
}

const __FlashStringHelper* classifyLevel(int value) {
  if (value < 80) return F("NEAR LOW");
  if (value > 940) return F("NEAR HIGH");
  return F("MID ANALOG");
}

void explainValue(int value, int delta) {
  Serial.print(F("VALUE="));
  Serial.print(value);
  Serial.print(F(" | DELTA="));
  Serial.print(delta);
  Serial.print(F(" | TYPE_HINT="));
  Serial.print(classifyLevel(value));

  if (value < 80) {
    Serial.print(F(" | INTERPRETATION=looks like digital LOW / magnet detected"));
  } else if (value > 940) {
    Serial.print(F(" | INTERPRETATION=looks like digital HIGH / clear"));
  } else if (delta > 25 || delta < -25) {
    Serial.print(F(" | INTERPRETATION=changing analog field detected"));
  } else {
    Serial.print(F(" | INTERPRETATION=no clear magnetic response yet"));
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT);

  delay(300);
  printIntro();

  baselineValue = readFilteredAnalog();
  Serial.print(F("BASELINE="));
  Serial.println(baselineValue);
  Serial.println();
}

void loop() {
  unsigned long nowMs = millis();
  if (nowMs - lastReportMs < REPORT_INTERVAL_MS) return;
  lastReportMs = nowMs;

  int value = readFilteredAnalog();
  int delta = value - baselineValue;
  explainValue(value, delta);
}
