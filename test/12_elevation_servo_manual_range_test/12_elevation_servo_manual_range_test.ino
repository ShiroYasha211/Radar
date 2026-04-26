/*
  12_elevation_servo_manual_range_test
  Manual exact-angle calibration for both azimuth and elevation servos.
*/

#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define AZIMUTH_PIN 9
#define ELEVATION_PIN 5

#define ZERO_AZIMUTH_ANGLE 60
#define ZERO_ELEVATION_ANGLE 45

#define AZIMUTH_MIN_ANGLE 0
#define AZIMUTH_MAX_ANGLE 170
#define ELEVATION_MIN_ANGLE 20
#define ELEVATION_MAX_ANGLE 90

#define AZIMUTH_STEP_DELAY_MS 15
#define ELEVATION_STEP_DELAY_MS 15
#define STATUS_REPORT_INTERVAL_MS 250

Servo azimuthServo;
Servo elevationServo;

int currentAzimuth = ZERO_AZIMUTH_ANGLE;
int currentElevation = ZERO_ELEVATION_ANGLE;
unsigned long lastStatusMs = 0;

char commandBuffer[32];
byte commandLength = 0;

void reportStatus(const __FlashStringHelper* label) {
  Serial.print(label);
  Serial.print(F(" | AZ="));
  Serial.print(currentAzimuth);
  Serial.print(F(" deg | EL="));
  Serial.print(currentElevation);
  Serial.println(F(" deg"));
}

void printIntro() {
  Serial.println(F("START: 12_elevation_servo_manual_range_test"));
  Serial.println(F("PART: Manual exact-angle control for azimuth + elevation servos"));
  Serial.println(F("PINS: AZIMUTH=D9, ELEVATION=D5"));
  Serial.println(F("POWER: use external 5V/6V supply with shared GND"));
  Serial.println(F("GOAL: type the exact angle and verify the real mechanical position"));
  Serial.println(F("RANGE: AZ=0..170 deg, EL=20..90 deg"));
  Serial.println();
}

void printHelp() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  a90      -> move azimuth to 90 degrees"));
  Serial.println(F("  e45      -> move elevation to 45 degrees"));
  Serial.println(F("  home     -> return both servos to zero/home"));
  Serial.println(F("  s        -> show current angles"));
  Serial.println(F("  ?        -> show help"));
  Serial.println();
}

void moveAzimuthSmooth(int targetAngle) {
  targetAngle = constrain(targetAngle, AZIMUTH_MIN_ANGLE, AZIMUTH_MAX_ANGLE);
  if (targetAngle == currentAzimuth) {
    reportStatus(F("AZ HOLD"));
    return;
  }

  int step = (targetAngle > currentAzimuth) ? 1 : -1;
  while (currentAzimuth != targetAngle) {
    currentAzimuth += step;
    azimuthServo.write(currentAzimuth);
    reportStatus(F("AZ MOVE"));
    delay(AZIMUTH_STEP_DELAY_MS);
  }
}

void moveElevationSmooth(int targetAngle) {
  targetAngle = constrain(targetAngle, ELEVATION_MIN_ANGLE, ELEVATION_MAX_ANGLE);
  if (targetAngle == currentElevation) {
    reportStatus(F("EL HOLD"));
    return;
  }

  int step = (targetAngle > currentElevation) ? 1 : -1;
  while (currentElevation != targetAngle) {
    currentElevation += step;
    elevationServo.write(currentElevation);
    reportStatus(F("EL MOVE"));
    delay(ELEVATION_STEP_DELAY_MS);
  }
}

void moveHome() {
  moveAzimuthSmooth(ZERO_AZIMUTH_ANGLE);
  moveElevationSmooth(ZERO_ELEVATION_ANGLE);
  reportStatus(F("HOME"));
}

bool isWhitespaceChar(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

char* trimCommand(char* text) {
  while (*text && isWhitespaceChar(*text)) {
    text++;
  }

  int len = strlen(text);
  while (len > 0 && isWhitespaceChar(text[len - 1])) {
    text[len - 1] = '\0';
    len--;
  }

  return text;
}

void normalizeCommand(char* text) {
  for (int i = 0; text[i] != '\0'; i++) {
    text[i] = (char)tolower((unsigned char)text[i]);
  }
}

void handleCommand(char* rawCommand) {
  char* command = trimCommand(rawCommand);
  normalizeCommand(command);
  if (command[0] == '\0') return;

  if (strcmp(command, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(command, "s") == 0 || strcmp(command, "status") == 0) {
    reportStatus(F("STATUS"));
    return;
  }

  if (strcmp(command, "home") == 0 || strcmp(command, "h") == 0) {
    moveHome();
    return;
  }

  if (command[0] == 'a') {
    moveAzimuthSmooth(atoi(command + 1));
    reportStatus(F("AZ SET"));
    return;
  }

  if (command[0] == 'e') {
    moveElevationSmooth(atoi(command + 1));
    reportStatus(F("EL SET"));
    return;
  }

  Serial.println(F("UNKNOWN COMMAND"));
  printHelp();
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

  azimuthServo.attach(AZIMUTH_PIN);
  elevationServo.attach(ELEVATION_PIN);
  azimuthServo.write(ZERO_AZIMUTH_ANGLE);
  elevationServo.write(ZERO_ELEVATION_ANGLE);
  currentAzimuth = ZERO_AZIMUTH_ANGLE;
  currentElevation = ZERO_ELEVATION_ANGLE;

  delay(900);
  printIntro();
  printHelp();
  reportStatus(F("HOME"));
}

void loop() {
  readSerialCommands();

  unsigned long nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_REPORT_INTERVAL_MS) {
    lastStatusMs = nowMs;
    reportStatus(F("LIVE"));
  }
}
