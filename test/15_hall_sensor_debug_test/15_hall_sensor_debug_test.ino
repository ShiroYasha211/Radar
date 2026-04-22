/*
  15_hall_sensor_debug_test
  Deep diagnostic test for Hall sensor / D10 input path.
*/

#define HALL_PIN 10
#define STATUS_LED_PIN LED_BUILTIN
#define ACTIVE_STATE LOW
#define REPORT_INTERVAL_MS 200
#define HINT_INTERVAL_MS 3000
#define EDGE_DEBOUNCE_MS 10

unsigned long lastReportMs = 0;
unsigned long lastHintMs = 0;
unsigned long lastEdgeMs = 0;
unsigned long detectCount = 0;
unsigned long clearCount = 0;

bool lastRawState = HIGH;
bool stableState = HIGH;

void printIntro() {
  Serial.println(F("START: 15_hall_sensor_debug_test"));
  Serial.println(F("PIN: D10"));
  Serial.println(F("MODE: INPUT_PULLUP"));
  Serial.println(F("LOGIC: LOW = DETECTED, HIGH = CLEAR"));
  Serial.println();
  Serial.println(F("STEP 1: leave the Hall sensor disconnected from D10."));
  Serial.println(F("STEP 2: briefly connect D10 directly to GND."));
  Serial.println(F("EXPECTED: state must become LOW / DETECTED immediately."));
  Serial.println(F("IF THAT FAILS: issue is not the Hall sensor. It is wiring, board pin, or test setup."));
  Serial.println();
  Serial.println(F("STEP 3: reconnect the Hall sensor."));
  Serial.println(F("WIRE IT AS: VCC->5V, GND->GND, OUT->D10."));
  Serial.println(F("STEP 4: move a strong magnet very close, then flip magnet polarity and try again."));
  Serial.println(F("IF D10 reacts to GND but not to the magnet: sensor pinout, sensor type, or magnet polarity is wrong."));
  Serial.println();
}

void reportState(const __FlashStringHelper* label) {
  Serial.print(label);
  Serial.print(F(" | RAW="));
  Serial.print(stableState == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F(" | STATE="));
  Serial.print(stableState == ACTIVE_STATE ? F("DETECTED") : F("CLEAR"));
  Serial.print(F(" | DETECT_COUNT="));
  Serial.print(detectCount);
  Serial.print(F(" | CLEAR_COUNT="));
  Serial.println(clearCount);
}

void printHint() {
  Serial.println(F("CHECKLIST:"));
  Serial.println(F("- If D10 to GND does not change to LOW: issue is board pin / wiring / wrong test setup."));
  Serial.println(F("- If D10 to GND works, but sensor never changes: issue is sensor wiring / wrong pinout / weak magnet / wrong polarity / wrong sensor type."));
  Serial.println(F("- Do NOT power the Hall sensor from 5.5V. Use stable 5V only."));
  Serial.println();
}

void updateStableState() {
  bool raw = digitalRead(HALL_PIN);
  unsigned long nowMs = millis();

  if (raw != lastRawState) {
    lastRawState = raw;
    lastEdgeMs = nowMs;
  }

  if (raw != stableState && nowMs - lastEdgeMs >= EDGE_DEBOUNCE_MS) {
    stableState = raw;
    if (stableState == ACTIVE_STATE) {
      detectCount++;
      digitalWrite(STATUS_LED_PIN, HIGH);
      reportState(F("EDGE"));
    } else {
      clearCount++;
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

  stableState = digitalRead(HALL_PIN);
  lastRawState = stableState;

  delay(300);
  printIntro();
  reportState(F("START"));
}

void loop() {
  updateStableState();

  unsigned long nowMs = millis();
  if (nowMs - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = nowMs;
    reportState(F("LIVE"));
  }

  if (nowMs - lastHintMs >= HINT_INTERVAL_MS) {
    lastHintMs = nowMs;
    printHint();
  }
}
