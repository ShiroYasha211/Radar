/*
  18_ultrasonic_range_quality_test
  Isolated ultrasonic sensor range and stability test.

  Wiring:
    TRIG -> D8
    ECHO -> A4
    VCC  -> 5V
    GND  -> GND

  Serial Monitor:
    115200 baud
    Newline setting is not important.
*/

#define TRIG_PIN 8
#define ECHO_PIN A4

const unsigned long ECHO_TIMEOUT_US = 35000UL;
const byte SAMPLE_COUNT = 15;
const unsigned int SAMPLE_GAP_MS = 70;
const unsigned int REPORT_GAP_MS = 350;
const float SOUND_CM_PER_US = 0.0343f;

float validDistances[SAMPLE_COUNT];

float durationToCm(unsigned long durationUs) {
  return (float)durationUs * SOUND_CM_PER_US * 0.5f;
}

unsigned long readEchoDuration() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
}

void sortFloatArray(float* values, byte count) {
  for (byte i = 0; i < count; i++) {
    for (byte j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }
}

void printIntro() {
  Serial.println(F("START: 18_ultrasonic_range_quality_test"));
  Serial.println(F("PART: Ultrasonic distance sensor only"));
  Serial.println(F("PINS: TRIG=D8, ECHO=A4"));
  Serial.println(F("BAUD: 115200"));
  Serial.println(F("TIMEOUT_US: 35000 (~600 cm theoretical echo window)"));
  Serial.println(F("REPORT: valid/no_echo, last_cm, median_cm, avg_cm, min_cm, max_cm, spread_cm"));
  Serial.println(F("NOTE: The real useful range is where valid readings stay stable and follow the tape-measure distance."));
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  delay(500);
  printIntro();
}

void loop() {
  byte validCount = 0;
  byte noEchoCount = 0;
  float lastDistance = -1.0f;

  for (byte i = 0; i < SAMPLE_COUNT; i++) {
    unsigned long duration = readEchoDuration();

    if (duration == 0) {
      noEchoCount++;
    } else {
      float distance = durationToCm(duration);
      validDistances[validCount] = distance;
      lastDistance = distance;
      validCount++;
    }

    delay(SAMPLE_GAP_MS);
  }

  Serial.print(F("valid="));
  Serial.print(validCount);
  Serial.print(F("/"));
  Serial.print(SAMPLE_COUNT);
  Serial.print(F(", no_echo="));
  Serial.print(noEchoCount);

  if (validCount == 0) {
    Serial.println(F(", result=NO_ECHO"));
    delay(REPORT_GAP_MS);
    return;
  }

  sortFloatArray(validDistances, validCount);

  float sum = 0.0f;
  for (byte i = 0; i < validCount; i++) {
    sum += validDistances[i];
  }

  float minCm = validDistances[0];
  float maxCm = validDistances[validCount - 1];
  float medianCm;
  if ((validCount % 2) == 1) {
    medianCm = validDistances[validCount / 2];
  } else {
    medianCm = (validDistances[(validCount / 2) - 1] + validDistances[validCount / 2]) * 0.5f;
  }

  float avgCm = sum / (float)validCount;
  float spreadCm = maxCm - minCm;

  Serial.print(F(", last_cm="));
  Serial.print(lastDistance, 1);
  Serial.print(F(", median_cm="));
  Serial.print(medianCm, 1);
  Serial.print(F(", avg_cm="));
  Serial.print(avgCm, 1);
  Serial.print(F(", min_cm="));
  Serial.print(minCm, 1);
  Serial.print(F(", max_cm="));
  Serial.print(maxCm, 1);
  Serial.print(F(", spread_cm="));
  Serial.print(spreadCm, 1);

  if (validCount >= 12 && spreadCm <= 8.0f) {
    Serial.println(F(", quality=STABLE"));
  } else if (validCount >= 8 && spreadCm <= 20.0f) {
    Serial.println(F(", quality=USABLE"));
  } else {
    Serial.println(F(", quality=WEAK_OR_NOISY"));
  }

  delay(REPORT_GAP_MS);
}
