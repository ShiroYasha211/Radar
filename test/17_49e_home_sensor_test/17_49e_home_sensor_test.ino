/*
  17_49e_home_sensor_test
  Calibration and home-detection test for 49E / 49E513 analog Hall sensor.
*/

#define HALL_PIN A5
#define SAMPLE_COUNT 9
#define REPORT_INTERVAL_MS 200
#define STATUS_LED_PIN LED_BUILTIN
#define HOME_DETECT_THRESHOLD 500
#define HOME_RELEASE_THRESHOLD 490
#define HOME_STABLE_HITS 2

int rawValue = 0;
int currentValue = 0;
bool homeDetected = false;
bool filterInitialized = false;
byte detectHits = 0;
byte clearHits = 0;

char commandBuffer[32];
byte commandLength = 0;
unsigned long lastReportMs = 0;

void sortSmallIntArray(int* values, byte count) {
  for (byte i = 0; i < count; i++) {
    for (byte j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        int temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }
}

int readRobustRaw() {
  int samples[SAMPLE_COUNT];
  for (byte i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = analogRead(HALL_PIN);
    delayMicroseconds(250);
  }

  sortSmallIntArray(samples, SAMPLE_COUNT);

  long sum = 0;
  for (byte i = 1; i < SAMPLE_COUNT - 1; i++) {
    sum += samples[i];
  }

  return (int)(sum / (SAMPLE_COUNT - 2));
}

int filterLinearValue(int current, int measured, byte numerator, byte denominator) {
  long scaled = (long)(measured - current) * numerator;
  if (scaled > 0) scaled += (denominator - 1);
  else if (scaled < 0) scaled -= (denominator - 1);
  return current + (int)(scaled / denominator);
}

void updateDetectionState() {
  rawValue = readRobustRaw();

  if (!filterInitialized) {
    currentValue = rawValue;
    filterInitialized = true;
  } else {
    currentValue = filterLinearValue(currentValue, rawValue, 3, 10);
  }

  if (currentValue >= HOME_DETECT_THRESHOLD) {
    if (detectHits < 250) detectHits++;
    clearHits = 0;
    if (detectHits >= HOME_STABLE_HITS) {
      homeDetected = true;
    }
  } else if (currentValue <= HOME_RELEASE_THRESHOLD) {
    if (clearHits < 250) clearHits++;
    detectHits = 0;
    if (clearHits >= HOME_STABLE_HITS) {
      homeDetected = false;
    }
  }
}

void reportState(const __FlashStringHelper* label) {
  Serial.print(label);
  Serial.print(F(" | RAW="));
  Serial.print(rawValue);
  Serial.print(F(" | FILTERED="));
  Serial.print(currentValue);
  Serial.print(F(" | TARGET="));
  Serial.println(homeDetected ? F("DETECTED") : F("CLEAR"));
}

void printIntro() {
  Serial.println(F("START: 17_49e_home_sensor_test"));
  Serial.println(F("PART: 49E / 49E513 analog Hall sensor"));
  Serial.println(F("PIN: A5"));
  Serial.println(F("USE A5 in the main project, not D10, because this sensor is analog."));
  Serial.print(F("DETECT: FILTERED >= "));
  Serial.println(HOME_DETECT_THRESHOLD);
  Serial.print(F("RELEASE: FILTERED <= "));
  Serial.println(HOME_RELEASE_THRESHOLD);
  Serial.println();
  Serial.println(F("WIRING:"));
  Serial.println(F("  49E VCC -> 5V"));
  Serial.println(F("  49E GND -> GND"));
  Serial.println(F("  49E OUT -> A5"));
  Serial.println();
  Serial.println(F("RESULT: the monitor will show RAW, FILTERED, and whether home is detected."));
  Serial.println();
}

void handleCommand(char* command) {
  if (command[0] == '\0') return;

  if (strcmp(command, "s") == 0 || strcmp(command, "status") == 0) {
    reportState(F("STATUS"));
    return;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      commandBuffer[0] = '\0';
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  delay(300);
  printIntro();

  updateDetectionState();
  reportState(F("START"));
}

void loop() {
  readSerialCommands();

  unsigned long nowMs = millis();
  if (nowMs - lastReportMs < REPORT_INTERVAL_MS) return;
  lastReportMs = nowMs;

  updateDetectionState();
  digitalWrite(STATUS_LED_PIN, homeDetected ? HIGH : LOW);

  reportState(F("LIVE"));
}
