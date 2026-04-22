/*
  14_hall_home_switch_test
  Isolated home/reference test for A3144 Hall effect sensor.
*/

#define HALL_PIN 10
#define STATUS_LED_PIN LED_BUILTIN
#define ACTIVE_STATE LOW
#define STATUS_REPORT_INTERVAL_MS 250
#define DEBOUNCE_MS 20

unsigned long lastStatusMs = 0;
unsigned long lastChangeMs = 0;
bool lastRawState = HIGH;
bool stableState = HIGH;
unsigned long triggerCount = 0;

bool readHallRaw() {
  return digitalRead(HALL_PIN);
}

bool hallTriggered() {
  return stableState == ACTIVE_STATE;
}

void printIntro() {
  Serial.println(F("START: 14_hall_home_switch_test"));
  Serial.println(F("PART: A3144 Hall effect home/reference sensor"));
  Serial.println(F("PIN: HALL_PIN=D10"));
  Serial.println(F("LOGIC: ACTIVE LOW when magnet is detected"));
  Serial.println(F("PASS CHECK: clean switch between DETECTED and CLEAR as the magnet passes the sensor"));
  Serial.println(F("FAIL SYMPTOM: always HIGH, always LOW, random flicker, no response to magnet"));
  Serial.println(F("TIP: rotate the magnet polarity if nothing happens"));
  Serial.println();
}

void printWiring() {
  Serial.println(F("WIRING OPTION 1 (Recommended): use Arduino INPUT_PULLUP, no external resistor needed"));
  Serial.println(F("  A3144 VCC -> 5V"));
  Serial.println(F("  A3144 GND -> GND"));
  Serial.println(F("  A3144 OUT -> D10"));
  Serial.println();
  Serial.println(F("WIRING OPTION 2: use your 10k resistor as pull-up"));
  Serial.println(F("  A3144 VCC -> 5V"));
  Serial.println(F("  A3144 GND -> GND"));
  Serial.println(F("  A3144 OUT -> D10"));
  Serial.println(F("  10k resistor between OUT and 5V"));
  Serial.println();
}

void reportState(const __FlashStringHelper* label) {
  Serial.print(label);
  Serial.print(F(" | HALL="));
  Serial.print(hallTriggered() ? F("DETECTED") : F("CLEAR"));
  Serial.print(F(" | RAW="));
  Serial.print(stableState == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F(" | COUNT="));
  Serial.println(triggerCount);
}

void updateHallState() {
  bool rawState = readHallRaw();
  unsigned long nowMs = millis();

  if (rawState != lastRawState) {
    lastRawState = rawState;
    lastChangeMs = nowMs;
  }

  if (stableState != rawState && nowMs - lastChangeMs >= DEBOUNCE_MS) {
    stableState = rawState;
    if (hallTriggered()) {
      triggerCount++;
      digitalWrite(STATUS_LED_PIN, HIGH);
      reportState(F("EDGE"));
    } else {
      digitalWrite(STATUS_LED_PIN, LOW);
      reportState(F("EDGE"));
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  stableState = readHallRaw();
  lastRawState = stableState;

  delay(300);
  printIntro();
  printWiring();
  reportState(F("START"));
}

void loop() {
  updateHallState();

  unsigned long nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_REPORT_INTERVAL_MS) {
    lastStatusMs = nowMs;
    reportState(F("LIVE"));
  }
}
