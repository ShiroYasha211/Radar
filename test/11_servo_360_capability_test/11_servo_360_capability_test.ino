/*
  11_servo_360_capability_test
  Diagnostic test to determine whether the connected servo is:
  1) a standard positional servo, or
  2) a continuous rotation servo.

  IMPORTANT:
  - Remove the horn/linkage before this test if possible.
  - Use external 5V/6V supply with shared GND.
  - Default pin is D9. Change SERVO_PIN to D5 if you want to test the elevation servo.
*/

#include <Servo.h>

#define SERVO_PIN 9
#define SERIAL_BAUD 115200

#define NEUTRAL_US 1500
#define CW_SLOW_US 1600
#define CW_FAST_US 1800
#define CCW_SLOW_US 1400
#define CCW_FAST_US 1200

#define POS_MIN_ANGLE 0
#define POS_CENTER_ANGLE 90
#define POS_MAX_ANGLE 180

#define HOLD_MS 1800

Servo testServo;
String inputBuffer = "";

void printIntro() {
  Serial.println(F("START: 11_servo_360_capability_test"));
  Serial.println(F("PART: Servo 360 capability check"));
  Serial.println(F("PINS: SERVO_PIN=D9 by default"));
  Serial.println(F("POWER: use external 5V/6V supply with shared GND"));
  Serial.println(F("IMPORTANT: remove the horn/linkage before test if possible"));
  Serial.println(F("GOAL: determine if the servo is standard positional or continuous rotation"));
  Serial.println();
  Serial.println(F("HOW TO INTERPRET RESULT:"));
  Serial.println(F("- If it moves to an angle and stops: standard positional servo"));
  Serial.println(F("- If it keeps spinning while the command is held: continuous rotation servo"));
  Serial.println(F("- A standard servo CANNOT do true 360 degree positioning"));
  Serial.println(F("- A continuous rotation servo CAN rotate continuously, but CANNOT hold a precise angle like 37 deg"));
  Serial.println();
}

void printHelp() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  auto      -> run full diagnostic cycle"));
  Serial.println(F("  neutral   -> send 1500us / center-stop signal"));
  Serial.println(F("  cwslow    -> continuous rotation test clockwise slow"));
  Serial.println(F("  cwfast    -> continuous rotation test clockwise fast"));
  Serial.println(F("  ccwslow   -> continuous rotation test counter-clockwise slow"));
  Serial.println(F("  ccwfast   -> continuous rotation test counter-clockwise fast"));
  Serial.println(F("  pos0      -> positional test to 0 deg"));
  Serial.println(F("  pos90     -> positional test to 90 deg"));
  Serial.println(F("  pos180    -> positional test to 180 deg"));
  Serial.println(F("  stop      -> same as neutral"));
  Serial.println(F("  ?         -> show help"));
  Serial.println();
}

void sendMicrosecondsCommand(int us, const __FlashStringHelper* label, unsigned long holdMs) {
  Serial.print(F("RUNNING: "));
  Serial.println(label);
  Serial.print(F("SIGNAL: "));
  Serial.print(us);
  Serial.println(F(" us"));
  testServo.writeMicroseconds(us);
  delay(holdMs);
}

void sendAngleCommand(int angle, const __FlashStringHelper* label, unsigned long holdMs) {
  Serial.print(F("RUNNING: "));
  Serial.println(label);
  Serial.print(F("ANGLE: "));
  Serial.print(angle);
  Serial.println(F(" deg"));
  testServo.write(angle);
  delay(holdMs);
}

void runAutoDiagnostic() {
  Serial.println(F("AUTO: Stage 1 -> positional commands"));
  sendAngleCommand(POS_CENTER_ANGLE, F("move to 90 deg"), HOLD_MS);
  sendAngleCommand(POS_MIN_ANGLE, F("move to 0 deg"), HOLD_MS);
  sendAngleCommand(POS_MAX_ANGLE, F("move to 180 deg"), HOLD_MS);
  sendAngleCommand(POS_CENTER_ANGLE, F("return to 90 deg"), HOLD_MS);

  Serial.println(F("OBSERVE: if the servo simply goes to each angle and stops, it is a standard servo"));
  Serial.println();

  Serial.println(F("AUTO: Stage 2 -> continuous rotation style commands"));
  sendMicrosecondsCommand(NEUTRAL_US, F("neutral / stop test"), HOLD_MS);
  sendMicrosecondsCommand(CW_SLOW_US, F("clockwise slow test"), HOLD_MS);
  sendMicrosecondsCommand(CW_FAST_US, F("clockwise fast test"), HOLD_MS);
  sendMicrosecondsCommand(NEUTRAL_US, F("neutral / stop test"), HOLD_MS);
  sendMicrosecondsCommand(CCW_SLOW_US, F("counter-clockwise slow test"), HOLD_MS);
  sendMicrosecondsCommand(CCW_FAST_US, F("counter-clockwise fast test"), HOLD_MS);
  sendMicrosecondsCommand(NEUTRAL_US, F("neutral / stop test"), HOLD_MS);

  Serial.println(F("RESULT GUIDE:"));
  Serial.println(F("- Standard servo: it will only twitch or move to a limited angle, not keep spinning"));
  Serial.println(F("- Continuous servo: it will keep spinning while CW/CCW command is active"));
  Serial.println(F("- If it is standard, true 360 degree rotation is NOT possible with this servo"));
  Serial.println(F("- If it is continuous, you lose exact angle targeting and it will not suit your radar turret aiming"));
  Serial.println(F("AUTO COMPLETE"));
  Serial.println();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "auto") {
    runAutoDiagnostic();
    return;
  }

  if (cmd == "neutral" || cmd == "stop") {
    sendMicrosecondsCommand(NEUTRAL_US, F("neutral / stop"), HOLD_MS);
    return;
  }

  if (cmd == "cwslow") {
    sendMicrosecondsCommand(CW_SLOW_US, F("clockwise slow"), HOLD_MS);
    return;
  }

  if (cmd == "cwfast") {
    sendMicrosecondsCommand(CW_FAST_US, F("clockwise fast"), HOLD_MS);
    return;
  }

  if (cmd == "ccwslow") {
    sendMicrosecondsCommand(CCW_SLOW_US, F("counter-clockwise slow"), HOLD_MS);
    return;
  }

  if (cmd == "ccwfast") {
    sendMicrosecondsCommand(CCW_FAST_US, F("counter-clockwise fast"), HOLD_MS);
    return;
  }

  if (cmd == "pos0") {
    sendAngleCommand(POS_MIN_ANGLE, F("move to 0 deg"), HOLD_MS);
    return;
  }

  if (cmd == "pos90") {
    sendAngleCommand(POS_CENTER_ANGLE, F("move to 90 deg"), HOLD_MS);
    return;
  }

  if (cmd == "pos180") {
    sendAngleCommand(POS_MAX_ANGLE, F("move to 180 deg"), HOLD_MS);
    return;
  }

  if (cmd == "?") {
    printHelp();
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
  Serial.begin(SERIAL_BAUD);
  testServo.attach(SERVO_PIN);
  delay(800);

  printIntro();
  printHelp();
  sendMicrosecondsCommand(NEUTRAL_US, F("initial neutral / safe command"), 600);
}

void loop() {
  readSerialCommands();
}
