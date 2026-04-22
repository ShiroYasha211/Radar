/*
  09_dual_servo_zero_test
  Combined azimuth + elevation servo test with home/zero return.
*/

#include <Servo.h>

#define AZIMUTH_PIN 9
#define ELEVATION_PIN 5

#define ZERO_AZIMUTH_ANGLE 60
#define ZERO_ELEVATION_ANGLE 45

#define AZIMUTH_MIN_ANGLE 0
#define AZIMUTH_MAX_ANGLE 130
#define ELEVATION_MIN_ANGLE 20
#define ELEVATION_MAX_ANGLE 90

#define AZIMUTH_STEP_DELAY_MS 15
#define ELEVATION_STEP_DELAY_MS 15
#define HOLD_DELAY_MS 1200

Servo azimuthServo;
Servo elevationServo;

int currentAzimuth = ZERO_AZIMUTH_ANGLE;
int currentElevation = ZERO_ELEVATION_ANGLE;
String inputBuffer = "";

void reportAngles(const __FlashStringHelper* label) {
  Serial.print(label);
  Serial.print(F(" | AZ="));
  Serial.print(currentAzimuth);
  Serial.print(F(" deg"));
  Serial.print(F(" | EL="));
  Serial.print(currentElevation);
  Serial.println(F(" deg"));
}

void printHelp() {
  Serial.println(F("MANUAL CONTROL COMMANDS:"));
  Serial.println(F("  a<number>  -> set azimuth angle, example: a90"));
  Serial.println(F("  e<number>  -> set elevation angle, example: e45"));
  Serial.println(F("  h          -> return both servos to home/zero"));
  Serial.println(F("  s          -> show current angles"));
  Serial.println(F("  run        -> auto test cycle"));
  Serial.println(F("  manual     -> stop auto cycle and stay in manual mode"));
  Serial.println(F("  ?          -> show help"));
  Serial.println();
}

void printIntro() {
  Serial.println(F("START: 09_dual_servo_zero_test"));
  Serial.println(F("PART: Dual servo home calibration"));
  Serial.println(F("PINS: AZIMUTH=D9, ELEVATION=D5"));
  Serial.println(F("POWER: use external 5V/6V supply with shared GND"));
  Serial.println(F("HOME: both servos return to ZERO_*_ANGLE on every startup and every cycle"));
  Serial.println(F("PASS CHECK: both servos return to the same home point every time"));
  Serial.println(F("FAIL SYMPTOM: home drift, chatter, wrong center, collision or brownout"));
  Serial.println(F("NOTE: if home is wrong, edit ZERO_AZIMUTH_ANGLE and ZERO_ELEVATION_ANGLE"));
  Serial.println();
}

void moveAzimuthSmooth(int targetAngle) {
  int step = (targetAngle >= currentAzimuth) ? 1 : -1;
  while (currentAzimuth != targetAngle) {
    currentAzimuth += step;
    azimuthServo.write(currentAzimuth);
    reportAngles(F("AZ MOVE"));
    delay(AZIMUTH_STEP_DELAY_MS);
  }
}

void moveElevationSmooth(int targetAngle) {
  int step = (targetAngle >= currentElevation) ? 1 : -1;
  while (currentElevation != targetAngle) {
    currentElevation += step;
    elevationServo.write(currentElevation);
    reportAngles(F("EL MOVE"));
    delay(ELEVATION_STEP_DELAY_MS);
  }
}

void moveHome() {
  moveAzimuthSmooth(ZERO_AZIMUTH_ANGLE);
  moveElevationSmooth(ZERO_ELEVATION_ANGLE);
}

bool autoMode = false;

void setAzimuthManual(int angle) {
  angle = constrain(angle, AZIMUTH_MIN_ANGLE, AZIMUTH_MAX_ANGLE);
  moveAzimuthSmooth(angle);
  reportAngles(F("AZ SET"));
}

void setElevationManual(int angle) {
  angle = constrain(angle, ELEVATION_MIN_ANGLE, ELEVATION_MAX_ANGLE);
  moveElevationSmooth(angle);
  reportAngles(F("EL SET"));
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "h") {
    moveHome();
    reportAngles(F("HOME"));
    return;
  }

  if (cmd == "s") {
    reportAngles(F("STATUS"));
    return;
  }

  if (cmd == "run") {
    autoMode = true;
    Serial.println(F("AUTO MODE ENABLED"));
    return;
  }

  if (cmd == "manual") {
    autoMode = false;
    Serial.println(F("MANUAL MODE ENABLED"));
    return;
  }

  if (cmd == "?") {
    printHelp();
    return;
  }

  if (cmd.startsWith("a")) {
    int angle = cmd.substring(1).toInt();
    setAzimuthManual(angle);
    return;
  }

  if (cmd.startsWith("e")) {
    int angle = cmd.substring(1).toInt();
    setElevationManual(angle);
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
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
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
  reportAngles(F("HOME"));
}

void loop() {
  readSerialCommands();
  if (!autoMode) return;

  Serial.println(F("RUNNING: move both servos to home/zero"));
  moveHome();
  reportAngles(F("HOME"));
  delay(HOLD_DELAY_MS);

  Serial.println(F("PASS CHECK: verify that the turret points to the real mechanical zero"));
  delay(1000);

  Serial.println(F("RUNNING: azimuth sweep around zero"));
  moveAzimuthSmooth(AZIMUTH_MIN_ANGLE);
  delay(HOLD_DELAY_MS);
  moveAzimuthSmooth(AZIMUTH_MAX_ANGLE);
  delay(HOLD_DELAY_MS);
  moveAzimuthSmooth(ZERO_AZIMUTH_ANGLE);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: elevation sweep around zero"));
  moveElevationSmooth(ELEVATION_MIN_ANGLE);
  delay(HOLD_DELAY_MS);
  moveElevationSmooth(ELEVATION_MAX_ANGLE);
  delay(HOLD_DELAY_MS);
  moveElevationSmooth(ZERO_ELEVATION_ANGLE);
  delay(HOLD_DELAY_MS);

  Serial.println(F("RUNNING: synchronized return to home"));
  moveHome();
  reportAngles(F("HOME"));
  Serial.println(F("PASS CHECK: both servos must always return to the exact same home point"));
  Serial.println(F("CYCLE COMPLETE"));
  Serial.println();
  delay(2000);
}
