#include <Servo.h>
#include <ctype.h>
#include <string.h>
#include <avr/wdt.h>

#define TRIG_PIN 8
#define ECHO_PIN A4

#define STEPPER_IN1 A0
#define STEPPER_IN2 A1
#define STEPPER_IN3 A2
#define STEPPER_IN4 A3

#define LAUNCH_AZIMUTH_PIN 9
#define LAUNCH_ELEVATION_PIN 5

#define MOTOR_IN1 4
#define MOTOR_IN2 2

#define RED_LED_PIN 3
#define YELLOW_LED_PIN 6
#define GREEN_LED_PIN 7
#define BUZZER_PIN 10
#define RADAR_HOME_SENSOR_PIN A5

#define ENABLE_AUTO_FIRE 1
#define DIAG_DISABLE_SENSOR 0
#define ENABLE_RADAR_HOME_SWITCH 1
#define ASSUME_STARTUP_FORWARD_REFERENCE 1

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

const long HALF_STEPS_PER_REV = 8192L;
const unsigned int STEPPER_SCAN_INTERVAL_US = 1600;
const byte MAX_STEPPER_LAG_STEPS = 2;
const byte MAX_STEPPER_CATCHUP_STEPS = 4;

const int BLIND_ZONE_MAX_DISTANCE = 40;
const int DANGER_MAX_DISTANCE = 200;
const int MID_MAX_DISTANCE = 300;
const int TARGET_DETECT_DISTANCE = 500;

const int TARGET_BIN_SIZE_DEG = 10;
const int TARGET_BIN_COUNT = 36;
const int TARGET_REINIT_DISTANCE_CM = 60;
const int TARGET_REINIT_ANGLE_DEG = 18;

const byte TARGET_CONFIDENCE_INIT = 16;
const byte TARGET_CONFIDENCE_MAX = 100;
const byte TARGET_CONFIDENCE_MIN_TRACK = TARGET_CONFIDENCE_INIT;
const byte TARGET_HITS_MIN_TRACK = 1;
const byte TARGET_CONFIDENCE_GAIN_STRONG = 12;
const byte TARGET_CONFIDENCE_GAIN_WEAK = 7;
const byte TARGET_CONFIDENCE_GAIN_POOR = 2;
const int TARGET_SCORE_DISTANCE_WEIGHT = 4;
const int TARGET_SCORE_CONFIDENCE_WEIGHT = 3;
const int TARGET_SCORE_HITS_WEIGHT = 5;
const int TARGET_SCORE_STABILITY_MAX_BONUS = 120;
const int TARGET_SCORE_DANGER_BONUS = 180;
const int TARGET_SCORE_AIMABLE_BONUS = 90;

const byte FILTER_DEN = 10;
const byte ANGLE_FILTER_NUM = 4;
const byte DIST_FILTER_NUM = 3;
const byte VELOCITY_FILTER_NUM = 3;
const int MAX_ANGULAR_VELOCITY_DPS = 240;
const int MAX_RADIAL_VELOCITY_CMPS = 250;

const int MIN_AZIMUTH = 0;
const int MAX_AZIMUTH = 170;
const int HOME_AZIMUTH = 60;
const bool AZIMUTH_DIRECTION_INVERTED = true;
const int RADAR_FORWARD_ANGLE = 0;
const int TURRET_FORWARD_RADAR_OFFSET_DEG = 0;
const int RADAR_AIM_LEFT_SPAN = 110;
const int RADAR_AIM_RIGHT_SPAN = 90;
const float SENSOR_HEIGHT_CM = 17.0f;
const float SENSOR_TO_LAUNCH_HEIGHT_DROP_CM = 9.0f;
const float LAUNCH_REFERENCE_HEIGHT_CM = SENSOR_HEIGHT_CM - SENSOR_TO_LAUNCH_HEIGHT_DROP_CM;
const float TURRET_OFFSET_FORWARD_CM = 8.0f;
const float TURRET_OFFSET_RIGHT_CM = 14.0f;
const float TARGET_REFERENCE_HEIGHT_CM = 25.0f;

const int MIN_ELEVATION = 20;
const int MAX_ELEVATION = 90;
const int DEFAULT_ELEVATION = 45;
const int TARGETING_ELEVATION = 55;
const float ELEVATION_OFFSET_CM = 8.0f;
const float ELEVATION_PLATFORM_BIAS_DEG = 0.0f;
const float ELEVATION_CLOSE_LIFT_DEG = 12.0f;
const float ELEVATION_DROP_GAIN_DEG_PER_M2 = 1.2f;
const float ELEVATION_APPROACH_GAIN_DEG_PER_CMPS = 0.015f;
const float DEG_PER_RAD = 57.2957795f;
const float AIM_SMOOTHING_ALPHA = 0.25f;
const float AIM_DAMPING = 0.22f;
const unsigned long AIM_PREDICTION_MS = 200;

const unsigned long SENSOR_MEASURE_INTERVAL_MS = 85;
const unsigned long TARGET_MEMORY_MS = 1200;
const unsigned long TARGET_PREDICTION_LIMIT_MS = 350;
const unsigned long AIM_HOME_RETURN_DELAY_MS = 4000;
const unsigned long ELEVATION_HOME_RETURN_DELAY_MS = 2000;
const unsigned long SERVO_UPDATE_INTERVAL_MS = 20;
const unsigned long SERIAL_UPDATE_INTERVAL_MS = SENSOR_MEASURE_INTERVAL_MS;
const unsigned long HOME_SWITCH_DEBOUNCE_MS = 40;
const unsigned long AZIMUTH_TRACK_LEAD_MS = 0;
const unsigned long ELEVATION_TRACK_LEAD_MS = 260;
const unsigned long FIRE_LOCK_HOLD_MS = 20;
const unsigned long FIRE_PULSE_MS = 600;
const unsigned long FIRE_COOLDOWN_MS = 0;

const int SERVO_AZIMUTH_STEP_DEG = 1;
const int SERVO_ELEVATION_STEP_DEG = 1;
const int SERVO_AZIMUTH_FAST_STEP_DEG = 4;
const int SERVO_ELEVATION_FAST_STEP_DEG = 3;
const float AZIMUTH_COMMAND_DEADBAND_DEG = 0.25f;
const float ELEVATION_COMMAND_DEADBAND_DEG = 0.35f;
const float SERVO_POSITION_DEADBAND_DEG = 0.35f;
const float AZIMUTH_FIRE_CORRECTION_DEG = -1.1f;
const float FIRE_AZIMUTH_TOLERANCE_DEG = 1.0f;
const float FIRE_ELEVATION_TOLERANCE_DEG = 2.0f;
const float AIM_COMMAND_LOCK_TOLERANCE_DEG = 0.8f;
const int SERVO_MIN_WRITE_ANGLE = 0;
const int SERVO_MAX_WRITE_ANGLE = 180;
const int AZIMUTH_SERVO_MIN_US = 544;
const int AZIMUTH_SERVO_MAX_US = 2400;
const int ELEVATION_SERVO_MIN_US = 544;
const int ELEVATION_SERVO_MAX_US = 2400;
const byte TARGET_AZIMUTH_FILTER_NUM = 5;
const byte TARGET_ELEVATION_FILTER_NUM = 1;
const unsigned long ECHO_TIMEOUT_US = 50000UL;
const byte HOME_SENSOR_SAMPLE_COUNT = 9;
const int HOME_SENSOR_DETECT_THRESHOLD = 600;
const int HOME_SENSOR_RELEASE_THRESHOLD = 560;
const byte HOME_SENSOR_STABLE_HITS = 2;

enum SystemState {
  IDLE_BLIND = 0,
  FAR_WARN = 1,
  MID_WARN = 2,
  DANGER_TRACK = 3
};

Servo launchAzimuth;
Servo launchElevation;

SystemState currentState = IDLE_BLIND;
SystemState lastAlertState = IDLE_BLIND;

int stepIndex = 0;
long currentHalfStepPos = 0;
int currentStepperAngle = 0;
int lastMeasurementAngle = 0;

int realDistance = -1;
int primaryTargetBin = -1;
int primaryTargetAngle = RADAR_FORWARD_ANGLE;
int primaryTargetDistance = -1;
int primaryTargetConfidence = 0;
int primaryTargetScore = 0;
int primaryTargetAngularVelocity = 0;
int primaryTargetRadialVelocity = 0;
byte primaryTargetHits = 0;
bool primaryTargetTrackable = false;
int activeVisibleTargets = 0;

int launchAzimuthAngle = HOME_AZIMUTH;
int launchElevationAngle = DEFAULT_ELEVATION;
int targetAzimuthAngle = HOME_AZIMUTH;
int targetElevationAngle = DEFAULT_ELEVATION;
float launchAzimuthPosition = HOME_AZIMUTH;
float launchElevationPosition = DEFAULT_ELEVATION;
float desiredAzimuthAngle = HOME_AZIMUTH;
float desiredElevationAngle = DEFAULT_ELEVATION;
float targetAzimuthCommand = HOME_AZIMUTH;
float targetElevationCommand = DEFAULT_ELEVATION;
float filteredAimForwardCm = 0.0f;
float filteredAimRightCm = 0.0f;
float filteredAimUpCm = 0.0f;
float previousAimForwardCm = 0.0f;
float previousAimRightCm = 0.0f;
float previousAimUpCm = 0.0f;
unsigned long lastAimFilterMs = 0;
bool aimFilterInitialized = false;

unsigned long lastStepperStepUs = 0;
unsigned long lastSensorMs = 0;
unsigned long lastServoMs = 0;
unsigned long lastSerialMs = 0;
unsigned long lastAlertToneMs = 0;
unsigned long lastHomeSyncMs = 0;
unsigned long lastTrackableAimMs = 0;
unsigned long fireLockStartMs = 0;
unsigned long firePulseEndMs = 0;
unsigned long lastFireMs = 0;
bool serialPacketPending = false;

bool stepperHolding = false;
bool stepperEnabled = true;
bool radarAngleKnown = false;
bool lastHomeSwitchActive = false;
bool homeCaptureArmed = true;
bool homeSensorDetected = false;
bool homeSensorInitialized = false;
bool firePulseActive = false;
bool fireCompletedForCurrentTarget = false;
int armedFireTargetBin = -1;
byte homeSensorDetectHits = 0;
byte homeSensorClearHits = 0;
int homeSensorFilteredValue = 0;

int targetBinDistance[TARGET_BIN_COUNT];
int targetBinAngle[TARGET_BIN_COUNT];
int targetBinAngularVelocity[TARGET_BIN_COUNT];
int targetBinRadialVelocity[TARGET_BIN_COUNT];
byte targetBinConfidence[TARGET_BIN_COUNT];
byte targetBinHits[TARGET_BIN_COUNT];
unsigned long targetBinTime[TARGET_BIN_COUNT];

char serialCommandBuffer[32];
byte serialCommandLength = 0;
byte startupResetCause = 0;

int wrapAngle360(int angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

int turretForwardRadarAngle() {
  return wrapAngle360(RADAR_FORWARD_ANGLE + TURRET_FORWARD_RADAR_OFFSET_DEG);
}

int angleDiffDegrees(int fromAngle, int toAngle) {
  int diff = wrapAngle360(toAngle) - wrapAngle360(fromAngle);
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

float wrapAngle360Float(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float angleDiffDegreesFloat(float fromAngle, float toAngle) {
  float diff = wrapAngle360Float(toAngle) - wrapAngle360Float(fromAngle);
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

float constrainFloatValue(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

int roundFloatToInt(float value) {
  if (value >= 0.0f) return (int)(value + 0.5f);
  return (int)(value - 0.5f);
}

int signedStep(int diff, byte numerator, byte denominator) {
  long scaled = (long)diff * numerator;
  if (scaled > 0) scaled += (denominator - 1);
  else if (scaled < 0) scaled -= (denominator - 1);

  int step = (int)(scaled / denominator);
  if (step == 0 && diff != 0) {
    step = diff > 0 ? 1 : -1;
  }
  return step;
}

int filterLinearValue(int currentValue, int measuredValue, byte numerator, byte denominator) {
  return currentValue + signedStep(measuredValue - currentValue, numerator, denominator);
}

int filterAngleValue(int currentAngle, int measuredAngle, byte numerator, byte denominator) {
  return wrapAngle360(currentAngle + signedStep(angleDiffDegrees(currentAngle, measuredAngle), numerator, denominator));
}

float smoothFloatValue(float previousValue, float measuredValue, float alpha) {
  return previousValue + alpha * (measuredValue - previousValue);
}

float dampFloatValue(float currentValue, float targetValue, float damping) {
  return currentValue + damping * (targetValue - currentValue);
}

int servoAngleToPulseUs(float angle, int minPulseUs, int maxPulseUs) {
  float constrainedAngle = constrainFloatValue(angle, (float)SERVO_MIN_WRITE_ANGLE, (float)SERVO_MAX_WRITE_ANGLE);
  float ratio = (constrainedAngle - SERVO_MIN_WRITE_ANGLE) / (float)(SERVO_MAX_WRITE_ANGLE - SERVO_MIN_WRITE_ANGLE);
  return minPulseUs + roundFloatToInt(ratio * (float)(maxPulseUs - minPulseUs));
}

void writeLaunchAzimuthServo(float angle) {
  launchAzimuth.writeMicroseconds(servoAngleToPulseUs(angle, AZIMUTH_SERVO_MIN_US, AZIMUTH_SERVO_MAX_US));
}

void writeLaunchElevationServo(float angle) {
  launchElevation.writeMicroseconds(servoAngleToPulseUs(angle, ELEVATION_SERVO_MIN_US, ELEVATION_SERVO_MAX_US));
}

void resetAimStabilizer() {
  aimFilterInitialized = false;
  lastAimFilterMs = 0;
  desiredAzimuthAngle = HOME_AZIMUTH;
  desiredElevationAngle = DEFAULT_ELEVATION;
  targetAzimuthCommand = HOME_AZIMUTH;
  targetElevationCommand = DEFAULT_ELEVATION;
}

void pauseAimStabilizer() {
  aimFilterInitialized = false;
  lastAimFilterMs = 0;
}

bool isVisibleDistance(int value) {
  return value > BLIND_ZONE_MAX_DISTANCE && value <= TARGET_DETECT_DISTANCE;
}

bool isBlindDistance(int value) {
  return value > 0 && value <= BLIND_ZONE_MAX_DISTANCE;
}

bool isAimableRadarAngle(int angle) {
  int relativeAngle = angleDiffDegrees(turretForwardRadarAngle(), angle);
  return relativeAngle >= -RADAR_AIM_LEFT_SPAN && relativeAngle <= RADAR_AIM_RIGHT_SPAN;
}

long angleToHalfStep(int angle) {
  long wrapped = wrapAngle360(angle);
  return (wrapped * HALF_STEPS_PER_REV + 180L) / 360L;
}

int halfStepToAngle(long pos) {
  while (pos < 0) pos += HALF_STEPS_PER_REV;
  while (pos >= HALF_STEPS_PER_REV) pos -= HALF_STEPS_PER_REV;
  return (int)((pos * 360L + HALF_STEPS_PER_REV / 2L) / HALF_STEPS_PER_REV) % 360;
}

float radarAngleToAzimuthFloat(float radarAngle) {
  float relativeAngle = angleDiffDegreesFloat((float)turretForwardRadarAngle(), radarAngle);
  float effectiveLeftSpan = (float)RADAR_AIM_LEFT_SPAN;
  float effectiveRightSpan = (float)RADAR_AIM_RIGHT_SPAN;
  if (AZIMUTH_DIRECTION_INVERTED) {
    relativeAngle = -relativeAngle;
    effectiveLeftSpan = RADAR_AIM_RIGHT_SPAN;
    effectiveRightSpan = RADAR_AIM_LEFT_SPAN;
  }
  relativeAngle = constrainFloatValue(relativeAngle, -effectiveLeftSpan, effectiveRightSpan);

  if (relativeAngle <= 0.0f) {
    float ratio = (relativeAngle + effectiveLeftSpan) / effectiveLeftSpan;
    float azimuth = MIN_AZIMUTH + ratio * (HOME_AZIMUTH - MIN_AZIMUTH);
    return constrainFloatValue(azimuth, (float)MIN_AZIMUTH, (float)HOME_AZIMUTH);
  }

  float ratio = relativeAngle / effectiveRightSpan;
  float azimuth = HOME_AZIMUTH + ratio * (MAX_AZIMUTH - HOME_AZIMUTH);
  return constrainFloatValue(azimuth, (float)HOME_AZIMUTH, (float)MAX_AZIMUTH);
}

int radarAngleToAzimuth(int radarAngle) {
  return constrain(
    roundFloatToInt(radarAngleToAzimuthFloat((float)radarAngle)),
    MIN_AZIMUTH,
    MAX_AZIMUTH
  );
}

float sensorDistanceToHorizontalProjection(int sensorDistance) {
  if (sensorDistance <= 0) return 1.0f;

  float sensorLineDistanceCm = (float)sensorDistance;
  float targetRelativeToSensorHeightCm = TARGET_REFERENCE_HEIGHT_CM - SENSOR_HEIGHT_CM;
  float verticalTerm = targetRelativeToSensorHeightCm * targetRelativeToSensorHeightCm;
  float lineTerm = sensorLineDistanceCm * sensorLineDistanceCm;

  if (lineTerm <= verticalTerm + 1.0f) {
    return 1.0f;
  }

  return sqrt(lineTerm - verticalTerm);
}

void radarAngleDistanceToLauncherVector(int radarAngle, int sensorDistance, float* forwardCm, float* rightCm, float* upCm) {
  float angleRad = (float)wrapAngle360(radarAngle) * PI / 180.0f;
  float horizontalProjectionCm = sensorDistanceToHorizontalProjection(sensorDistance);
  float targetForwardCm = cos(angleRad) * horizontalProjectionCm;
  float targetRightCm = sin(angleRad) * horizontalProjectionCm;

  *forwardCm = targetForwardCm - TURRET_OFFSET_FORWARD_CM;
  *rightCm = targetRightCm - TURRET_OFFSET_RIGHT_CM;
  *upCm = TARGET_REFERENCE_HEIGHT_CM - LAUNCH_REFERENCE_HEIGHT_CM;
}

int radarAngleDistanceToTurretAngle(int radarAngle, int sensorDistance) {
  float turretRelativeForwardCm;
  float turretRelativeRightCm;
  float turretRelativeUpCm;
  radarAngleDistanceToLauncherVector(radarAngle, sensorDistance, &turretRelativeForwardCm, &turretRelativeRightCm, &turretRelativeUpCm);

  float turretAngleDeg = atan2(turretRelativeRightCm, turretRelativeForwardCm) * DEG_PER_RAD;
  if (turretAngleDeg < 0.0f) turretAngleDeg += 360.0f;
  return wrapAngle360((int)(turretAngleDeg + 0.5f));
}

int radarAngleDistanceToTurretDistance(int radarAngle, int sensorDistance) {
  float turretRelativeForwardCm;
  float turretRelativeRightCm;
  float turretRelativeUpCm;
  radarAngleDistanceToLauncherVector(radarAngle, sensorDistance, &turretRelativeForwardCm, &turretRelativeRightCm, &turretRelativeUpCm);

  float turretDistanceCm = sqrt(turretRelativeForwardCm * turretRelativeForwardCm + turretRelativeRightCm * turretRelativeRightCm);
  if (turretDistanceCm < 1.0f) turretDistanceCm = 1.0f;
  if (turretDistanceCm > TARGET_DETECT_DISTANCE) turretDistanceCm = TARGET_DETECT_DISTANCE;
  return (int)(turretDistanceCm + 0.5f);
}

float launcherVectorToRadarAngleFloat(float forwardCm, float rightCm) {
  float turretAngleDeg = atan2(rightCm, forwardCm) * DEG_PER_RAD;
  if (turretAngleDeg < 0.0f) turretAngleDeg += 360.0f;
  return wrapAngle360Float(turretAngleDeg);
}

int launcherVectorToRadarAngle(float forwardCm, float rightCm) {
  return wrapAngle360(roundFloatToInt(launcherVectorToRadarAngleFloat(forwardCm, rightCm)));
}

int clampVelocity(int value, int maxAbsValue) {
  if (value > maxAbsValue) return maxAbsValue;
  if (value < -maxAbsValue) return -maxAbsValue;
  return value;
}

int dynamicServoStep(int errorMagnitude, int slowStep, int fastStep) {
  if (errorMagnitude >= 24) return fastStep;
  if (errorMagnitude >= 12) return max(slowStep + 2, fastStep - 1);
  if (errorMagnitude >= 6) return slowStep + 1;
  return slowStep;
}

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

int readHomeSensorRobustRaw() {
  int samples[HOME_SENSOR_SAMPLE_COUNT];
  for (byte i = 0; i < HOME_SENSOR_SAMPLE_COUNT; i++) {
    samples[i] = analogRead(RADAR_HOME_SENSOR_PIN);
    delayMicroseconds(250);
  }

  sortSmallIntArray(samples, HOME_SENSOR_SAMPLE_COUNT);

  long sum = 0;
  for (byte i = 1; i < HOME_SENSOR_SAMPLE_COUNT - 1; i++) {
    sum += samples[i];
  }

  return (int)(sum / (HOME_SENSOR_SAMPLE_COUNT - 2));
}

void serviceHomeSensor() {
#if ENABLE_RADAR_HOME_SWITCH
  int rawValue = readHomeSensorRobustRaw();

  if (!homeSensorInitialized) {
    homeSensorFilteredValue = rawValue;
    homeSensorInitialized = true;
  } else {
    homeSensorFilteredValue = filterLinearValue(homeSensorFilteredValue, rawValue, 3, FILTER_DEN);
  }

  if (homeSensorFilteredValue >= HOME_SENSOR_DETECT_THRESHOLD) {
    if (homeSensorDetectHits < 250) homeSensorDetectHits++;
    homeSensorClearHits = 0;
    if (homeSensorDetectHits >= HOME_SENSOR_STABLE_HITS) {
      homeSensorDetected = true;
    }
  } else if (homeSensorFilteredValue <= HOME_SENSOR_RELEASE_THRESHOLD) {
    if (homeSensorClearHits < 250) homeSensorClearHits++;
    homeSensorDetectHits = 0;
    if (homeSensorClearHits >= HOME_SENSOR_STABLE_HITS) {
      homeSensorDetected = false;
    }
  }
#endif
}

SystemState stateFromDistance(int distance) {
  if (distance > BLIND_ZONE_MAX_DISTANCE && distance <= DANGER_MAX_DISTANCE) return DANGER_TRACK;
  if (distance > DANGER_MAX_DISTANCE && distance <= MID_MAX_DISTANCE) return MID_WARN;
  if (distance > MID_MAX_DISTANCE && distance <= TARGET_DETECT_DISTANCE) return FAR_WARN;
  return IDLE_BLIND;
}

const char* stateName(SystemState state) {
  switch (state) {
    case FAR_WARN: return "FAR WARN";
    case MID_WARN: return "MID WARN";
    case DANGER_TRACK: return "DANGER TRACK";
    default: return "IDLE/BLIND";
  }
}

int predictionAgeMs(unsigned long nowMs, byte index) {
  unsigned long ageMs = nowMs - targetBinTime[index];
  if (ageMs > TARGET_PREDICTION_LIMIT_MS) ageMs = TARGET_PREDICTION_LIMIT_MS;
  return (int)ageMs;
}

int predictedAngleForBin(byte index, unsigned long nowMs) {
  int ageMs = predictionAgeMs(nowMs, index);
  long predicted = (long)targetBinAngle[index] + ((long)targetBinAngularVelocity[index] * ageMs) / 1000L;
  return wrapAngle360((int)predicted);
}

int predictedDistanceForBin(byte index, unsigned long nowMs) {
  int ageMs = predictionAgeMs(nowMs, index);
  long predicted = (long)targetBinDistance[index] + ((long)targetBinRadialVelocity[index] * ageMs) / 1000L;
  if (predicted < 1) predicted = 1;
  if (predicted > TARGET_DETECT_DISTANCE) predicted = TARGET_DETECT_DISTANCE;
  return (int)predicted;
}

int predictFutureAngle(int angle, int angularVelocity, unsigned long leadMs) {
  long predicted = (long)angle + ((long)angularVelocity * (long)leadMs) / 1000L;
  return wrapAngle360((int)predicted);
}

int predictFutureDistance(int distance, int radialVelocity, unsigned long leadMs) {
  long predicted = (long)distance + ((long)radialVelocity * (long)leadMs) / 1000L;
  if (predicted < 1) predicted = 1;
  if (predicted > TARGET_DETECT_DISTANCE) predicted = TARGET_DETECT_DISTANCE;
  return (int)predicted;
}

int targetStabilityBonus(byte index) {
  int angularPenalty = abs(targetBinAngularVelocity[index]) / 6;
  int radialPenalty = abs(targetBinRadialVelocity[index]) / 4;
  int bonus = TARGET_SCORE_STABILITY_MAX_BONUS - angularPenalty - radialPenalty;
  if (bonus < 0) bonus = 0;
  return bonus;
}

int scoreTargetBin(byte index, unsigned long nowMs, int predictedAngle, int predictedDistance, int effectiveConfidence) {
  unsigned long ageMs = nowMs - targetBinTime[index];
  int score = 0;
  score += (TARGET_DETECT_DISTANCE - predictedDistance) * TARGET_SCORE_DISTANCE_WEIGHT;
  score += effectiveConfidence * TARGET_SCORE_CONFIDENCE_WEIGHT;
  score += min((int)targetBinHits[index], 20) * TARGET_SCORE_HITS_WEIGHT;
  score += targetStabilityBonus(index);

  if (stateFromDistance(predictedDistance) == DANGER_TRACK) {
    score += TARGET_SCORE_DANGER_BONUS;
  }

  if (isAimableRadarAngle(predictedAngle)) {
    score += TARGET_SCORE_AIMABLE_BONUS;
  }

  score -= (int)(ageMs / 6UL);
  return score;
}

void clearTargetMap() {
  for (int i = 0; i < TARGET_BIN_COUNT; i++) {
    targetBinDistance[i] = -1;
    targetBinAngle[i] = i * TARGET_BIN_SIZE_DEG;
    targetBinAngularVelocity[i] = 0;
    targetBinRadialVelocity[i] = 0;
    targetBinConfidence[i] = 0;
    targetBinHits[i] = 0;
    targetBinTime[i] = 0;
  }
}

void resetTargetSelection() {
  primaryTargetBin = -1;
  primaryTargetAngle = turretForwardRadarAngle();
  primaryTargetDistance = -1;
  primaryTargetConfidence = 0;
  primaryTargetScore = 0;
  primaryTargetAngularVelocity = 0;
  primaryTargetRadialVelocity = 0;
  primaryTargetHits = 0;
  primaryTargetTrackable = false;
  activeVisibleTargets = 0;
  currentState = IDLE_BLIND;
}

void resetTrackingState() {
  clearTargetMap();
  resetTargetSelection();
  lastTrackableAimMs = 0;
  resetAimStabilizer();
  realDistance = -1;
}

void setRadarReferenceAngle(int radarAngle) {
  radarAngle = wrapAngle360(radarAngle);
  currentHalfStepPos = angleToHalfStep(radarAngle);
  currentStepperAngle = radarAngle;
  lastMeasurementAngle = radarAngle;
  radarAngleKnown = true;
  serialPacketPending = true;
}

void printSerialHelp() {
  Serial.println(F("# COMMANDS: zero | status | stepper on | stepper off | help"));
}

void printResetCause() {
  Serial.print(F("# RESET CAUSE: "));
  if (startupResetCause & _BV(BORF)) {
    Serial.println(F("BROWNOUT / POWER DROP"));
  } else if (startupResetCause & _BV(WDRF)) {
    Serial.println(F("WATCHDOG"));
  } else if (startupResetCause & _BV(EXTRF)) {
    Serial.println(F("EXTERNAL RESET"));
  } else if (startupResetCause & _BV(PORF)) {
    Serial.println(F("POWER ON"));
  } else {
    Serial.println(F("UNKNOWN"));
  }
}

void printRadarStatus() {
  Serial.print(F("# RADAR ANGLE="));
  Serial.print(currentStepperAngle);
  Serial.print(F(" | LAST_MEAS="));
  Serial.print(lastMeasurementAngle);
  Serial.print(F(" | KNOWN="));
  if (radarAngleKnown) Serial.print(F("YES"));
  else Serial.print(F("NO"));
  Serial.print(F(" | STEPPER="));
  if (stepperEnabled) Serial.print(F("ON"));
  else Serial.print(F("OFF"));
  Serial.print(F(" | TURRET_FWD="));
  Serial.print(turretForwardRadarAngle());
  Serial.print(F(" | OFFSET(cm) F="));
  Serial.print(TURRET_OFFSET_FORWARD_CM, 1);
  Serial.print(F(" R="));
  Serial.print(TURRET_OFFSET_RIGHT_CM, 1);
  Serial.print(F(" SENSOR_H="));
  Serial.print(SENSOR_HEIGHT_CM, 1);
  Serial.print(F(" LAUNCH_DROP="));
  Serial.print(SENSOR_TO_LAUNCH_HEIGHT_DROP_CM, 1);
  Serial.print(F(" LAUNCH_H="));
  Serial.print(LAUNCH_REFERENCE_HEIGHT_CM, 1);
  Serial.print(F(" TARGET_H="));
  Serial.print(TARGET_REFERENCE_HEIGHT_CM, 1);
  Serial.print(F(" | STATE="));
  Serial.print(stateName(currentState));
  Serial.print(F(" | HOME49E="));
#if ENABLE_RADAR_HOME_SWITCH
  Serial.print(homeSensorFilteredValue);
  Serial.print(homeSensorDetected ? F("(HOME)") : F("(CLEAR)"));
#else
  Serial.print(F("OFF"));
#endif
  Serial.print(F(" | TARGETS="));
  Serial.print(activeVisibleTargets);
  Serial.print(F(" | FIRE="));
  if (firePulseActive) Serial.print(F("PULSE"));
  else if (fireCompletedForCurrentTarget) Serial.print(F("DONE"));
  else Serial.print(F("READY"));
  Serial.print(F(" | TARGET="));
  if (primaryTargetDistance > 0) {
    Serial.print(primaryTargetAngle);
    Serial.print(F("/"));
    Serial.print(primaryTargetDistance);
    Serial.print(F(" | CONF="));
    Serial.print(primaryTargetConfidence);
    Serial.print(F(" | SCORE="));
    Serial.print(primaryTargetScore);
    Serial.print(F(" | VEL="));
    Serial.print(primaryTargetAngularVelocity);
    Serial.print(F("dps/"));
    Serial.print(primaryTargetRadialVelocity);
    Serial.print(F("cmps"));
  } else {
    Serial.print(F("---"));
  }
  Serial.println();
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

void handleSerialCommand(char* rawCommand) {
  char* command = trimCommand(rawCommand);
  normalizeCommand(command);
  if (command[0] == '\0') return;

  if (strcmp(command, "zero") == 0) {
    setRadarReferenceAngle(RADAR_FORWARD_ANGLE);
    resetTrackingState();
    Serial.println(F("# ZERO OK: current radar position is now the forward reference"));
    printRadarStatus();
    return;
  }

  if (strcmp(command, "status") == 0) {
    printRadarStatus();
    return;
  }

  if (strcmp(command, "stepper on") == 0) {
    stepperEnabled = true;
    lastStepperStepUs = micros();
    Serial.println(F("# STEPPER ENABLED"));
    return;
  }

  if (strcmp(command, "stepper off") == 0) {
    stepperEnabled = false;
    stepperOff();
    Serial.println(F("# STEPPER DISABLED"));
    return;
  }

  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printSerialHelp();
    return;
  }

  Serial.println(F("# UNKNOWN COMMAND"));
  printSerialHelp();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialCommandBuffer[serialCommandLength] = '\0';
      handleSerialCommand(serialCommandBuffer);
      serialCommandLength = 0;
      serialCommandBuffer[0] = '\0';
      continue;
    }

    if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength++] = c;
    }
  }
}

bool isHomeSwitchTriggered() {
#if ENABLE_RADAR_HOME_SWITCH
  return homeSensorDetected;
#else
  return false;
#endif
}

void syncHomeSwitchReferenceIfNeeded() {
#if ENABLE_RADAR_HOME_SWITCH
  bool homeActive = isHomeSwitchTriggered();
  unsigned long nowMs = millis();

  if (!homeActive) {
    homeCaptureArmed = true;
  }

  if (homeActive && homeCaptureArmed && nowMs - lastHomeSyncMs >= HOME_SWITCH_DEBOUNCE_MS) {
    setRadarReferenceAngle(RADAR_FORWARD_ANGLE);
    lastHomeSyncMs = nowMs;
    homeCaptureArmed = false;
    Serial.println(F("# HOME SENSOR: ZERO REFERENCE CAPTURED"));
  }

  lastHomeSwitchActive = homeActive;
#endif
}

void initializeRadarReference() {
#if ENABLE_RADAR_HOME_SWITCH
  for (byte i = 0; i < 4; i++) {
    serviceHomeSensor();
  }
  lastHomeSwitchActive = isHomeSwitchTriggered();
  if (lastHomeSwitchActive) {
    setRadarReferenceAngle(RADAR_FORWARD_ANGLE);
    homeCaptureArmed = false;
    Serial.println(F("# RADAR REFERENCE: 49E HOME SENSOR ACTIVE AT STARTUP"));
  } else {
    radarAngleKnown = false;
    currentHalfStepPos = 0;
    currentStepperAngle = 0;
    lastMeasurementAngle = 0;
    homeCaptureArmed = true;
    Serial.println(F("# RADAR REFERENCE: WAITING FOR 49E HOME SENSOR OR MANUAL 'zero'"));
  }
#elif ASSUME_STARTUP_FORWARD_REFERENCE
  setRadarReferenceAngle(RADAR_FORWARD_ANGLE);
  homeCaptureArmed = true;
  Serial.println(F("# RADAR REFERENCE: STARTUP POSITION ASSUMED FORWARD"));
#else
  radarAngleKnown = false;
  currentHalfStepPos = 0;
  currentStepperAngle = 0;
  lastMeasurementAngle = 0;
  homeCaptureArmed = true;
  Serial.println(F("# RADAR REFERENCE: UNKNOWN, USE SERIAL COMMAND 'zero'"));
#endif
}

void stopFire() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void startFirePulse() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

bool isAimLockedForFire() {
  if (!primaryTargetTrackable) return false;
  if (!aimFilterInitialized) return false;

  float azimuthPositionError = fabs(desiredAzimuthAngle - launchAzimuthPosition);
  float elevationPositionError = fabs(desiredElevationAngle - launchElevationPosition);
  float azimuthCommandError = fabs(desiredAzimuthAngle - targetAzimuthCommand);
  float elevationCommandError = fabs(desiredElevationAngle - targetElevationCommand);

  return
    azimuthPositionError <= FIRE_AZIMUTH_TOLERANCE_DEG &&
    elevationPositionError <= FIRE_ELEVATION_TOLERANCE_DEG &&
    azimuthCommandError <= AIM_COMMAND_LOCK_TOLERANCE_DEG &&
    elevationCommandError <= AIM_COMMAND_LOCK_TOLERANCE_DEG;
}

void serviceFireControl() {
#if ENABLE_AUTO_FIRE
  unsigned long nowMs = millis();

  if (!primaryTargetTrackable || primaryTargetBin < 0) {
    firePulseActive = false;
    fireCompletedForCurrentTarget = false;
    armedFireTargetBin = -1;
    fireLockStartMs = 0;
    stopFire();
    return;
  }

  if (primaryTargetBin != armedFireTargetBin) {
    armedFireTargetBin = primaryTargetBin;
    fireCompletedForCurrentTarget = false;
    firePulseActive = false;
    fireLockStartMs = 0;
    stopFire();
  }

  if (firePulseActive) {
    if ((long)(nowMs - firePulseEndMs) < 0) {
      startFirePulse();
      return;
    }

    firePulseActive = false;
    stopFire();
    return;
  }

  if (!isAimLockedForFire()) {
    fireLockStartMs = 0;
    stopFire();
    return;
  }

  if (fireLockStartMs == 0) {
    fireLockStartMs = nowMs;
    if (FIRE_LOCK_HOLD_MS > 0) {
      stopFire();
      return;
    }
  }

  if (fireCompletedForCurrentTarget) {
    stopFire();
    return;
  }

  if (nowMs - fireLockStartMs < FIRE_LOCK_HOLD_MS) {
    stopFire();
    return;
  }

  if (lastFireMs != 0 && nowMs - lastFireMs < FIRE_COOLDOWN_MS) {
    stopFire();
    return;
  }

  firePulseActive = true;
  fireCompletedForCurrentTarget = true;
  lastFireMs = nowMs;
  firePulseEndMs = nowMs + FIRE_PULSE_MS;
  startFirePulse();
#else
  stopFire();
#endif
}

void stepperOff() {
  if (!stepperHolding) return;
  for (int i = 0; i < 4; i++) {
    digitalWrite(stepperPins[i], LOW);
  }
  stepperHolding = false;
}

void applyHalfStep(int direction) {
  stepIndex += direction;
  if (stepIndex >= 8) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 7;

  for (int i = 0; i < 4; i++) {
    digitalWrite(stepperPins[i], halfStepSequence[stepIndex][i]);
  }

  currentHalfStepPos += direction;
  while (currentHalfStepPos >= HALF_STEPS_PER_REV) currentHalfStepPos -= HALF_STEPS_PER_REV;
  while (currentHalfStepPos < 0) currentHalfStepPos += HALF_STEPS_PER_REV;

  currentStepperAngle = halfStepToAngle(currentHalfStepPos);
  stepperHolding = true;
}

void serviceStepper() {
  if (!stepperEnabled) {
    stepperOff();
    return;
  }

  unsigned long nowUs = micros();
  if (lastStepperStepUs == 0) {
    lastStepperStepUs = nowUs;
    return;
  }

  unsigned long elapsedUs = (unsigned long)(nowUs - lastStepperStepUs);
  if (elapsedUs < STEPPER_SCAN_INTERVAL_US) {
    return;
  }

  if (elapsedUs > (unsigned long)STEPPER_SCAN_INTERVAL_US * MAX_STEPPER_LAG_STEPS) {
    lastStepperStepUs = nowUs - STEPPER_SCAN_INTERVAL_US;
    elapsedUs = STEPPER_SCAN_INTERVAL_US;
  }

  byte queuedSteps = 0;
  while (elapsedUs >= STEPPER_SCAN_INTERVAL_US && queuedSteps < MAX_STEPPER_CATCHUP_STEPS) {
    lastStepperStepUs += STEPPER_SCAN_INTERVAL_US;
    elapsedUs -= STEPPER_SCAN_INTERVAL_US;
    queuedSteps++;
  }

  while (queuedSteps > 0) {
    applyHalfStep(1);
    queuedSteps--;
  }

  syncHomeSwitchReferenceIfNeeded();
}

long readEchoDuration() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
}

void clearTargetBin(byte index) {
  targetBinDistance[index] = -1;
  targetBinAngularVelocity[index] = 0;
  targetBinRadialVelocity[index] = 0;
  targetBinConfidence[index] = 0;
  targetBinHits[index] = 0;
  targetBinTime[index] = 0;
}

void updateTargetMap(int angle, int distance) {
  if (!radarAngleKnown) return;
  if (!isVisibleDistance(distance)) return;

  int wrappedAngle = wrapAngle360(angle);
  int binIndex = ((wrappedAngle + (TARGET_BIN_SIZE_DEG / 2)) % 360) / TARGET_BIN_SIZE_DEG;
  unsigned long nowMs = millis();

  if (targetBinDistance[binIndex] < 0 || nowMs - targetBinTime[binIndex] > TARGET_MEMORY_MS) {
    targetBinDistance[binIndex] = distance;
    targetBinAngle[binIndex] = wrappedAngle;
    targetBinAngularVelocity[binIndex] = 0;
    targetBinRadialVelocity[binIndex] = 0;
    targetBinConfidence[binIndex] = TARGET_CONFIDENCE_INIT;
    targetBinHits[binIndex] = 1;
    targetBinTime[binIndex] = nowMs;
    return;
  }

  int prevAngle = targetBinAngle[binIndex];
  int prevDistance = targetBinDistance[binIndex];
  int angleError = abs(angleDiffDegrees(prevAngle, wrappedAngle));
  int distanceError = abs(prevDistance - distance);

  if (angleError > TARGET_REINIT_ANGLE_DEG || distanceError > TARGET_REINIT_DISTANCE_CM) {
    targetBinDistance[binIndex] = distance;
    targetBinAngle[binIndex] = wrappedAngle;
    targetBinAngularVelocity[binIndex] = 0;
    targetBinRadialVelocity[binIndex] = 0;
    targetBinConfidence[binIndex] = TARGET_CONFIDENCE_INIT;
    targetBinHits[binIndex] = 1;
    targetBinTime[binIndex] = nowMs;
    return;
  }

  unsigned long dtMs = nowMs - targetBinTime[binIndex];
  if (dtMs == 0) dtMs = 1;

  int observedAngularVelocity = (angleDiffDegrees(prevAngle, wrappedAngle) * 1000L) / (long)dtMs;
  int observedRadialVelocity = ((distance - prevDistance) * 1000L) / (long)dtMs;
  observedAngularVelocity = clampVelocity(observedAngularVelocity, MAX_ANGULAR_VELOCITY_DPS);
  observedRadialVelocity = clampVelocity(observedRadialVelocity, MAX_RADIAL_VELOCITY_CMPS);

  targetBinAngle[binIndex] = filterAngleValue(prevAngle, wrappedAngle, ANGLE_FILTER_NUM, FILTER_DEN);
  targetBinDistance[binIndex] = filterLinearValue(prevDistance, distance, DIST_FILTER_NUM, FILTER_DEN);
  targetBinAngularVelocity[binIndex] = filterLinearValue(targetBinAngularVelocity[binIndex], observedAngularVelocity, VELOCITY_FILTER_NUM, FILTER_DEN);
  targetBinRadialVelocity[binIndex] = filterLinearValue(targetBinRadialVelocity[binIndex], observedRadialVelocity, VELOCITY_FILTER_NUM, FILTER_DEN);

  int newConfidence = targetBinConfidence[binIndex];
  if (angleError <= 6 && distanceError <= 12) newConfidence += TARGET_CONFIDENCE_GAIN_STRONG;
  else if (angleError <= 12 && distanceError <= 24) newConfidence += TARGET_CONFIDENCE_GAIN_WEAK;
  else newConfidence += TARGET_CONFIDENCE_GAIN_POOR;

  if (newConfidence > TARGET_CONFIDENCE_MAX) newConfidence = TARGET_CONFIDENCE_MAX;
  targetBinConfidence[binIndex] = (byte)newConfidence;
  if (targetBinHits[binIndex] < 250) targetBinHits[binIndex]++;
  targetBinTime[binIndex] = nowMs;
}

void selectPrimaryTarget() {
  unsigned long nowMs = millis();
  int bestIndex = -1;
  int bestDistance = TARGET_DETECT_DISTANCE + 1;
  int bestConfidence = -1;
  int bestScore = -32767;
  unsigned long bestAge = 0xFFFFFFFFUL;
  int visibleTargets = 0;

  for (int i = 0; i < TARGET_BIN_COUNT; i++) {
    if (targetBinDistance[i] < 0) continue;

    unsigned long ageMs = nowMs - targetBinTime[i];
    if (ageMs > TARGET_MEMORY_MS) {
      clearTargetBin(i);
      continue;
    }

    int predictedDistance = predictedDistanceForBin(i, nowMs);
    if (!isVisibleDistance(predictedDistance)) {
      continue;
    }

    visibleTargets++;
    int effectiveConfidence = (int)targetBinConfidence[i] - (int)(ageMs / 80UL);
    if (effectiveConfidence < 0) effectiveConfidence = 0;
    int predictedAngle = predictedAngleForBin(i, nowMs);
    int score = scoreTargetBin(i, nowMs, predictedAngle, predictedDistance, effectiveConfidence);

    if (bestIndex < 0 ||
        score > bestScore ||
        (score == bestScore && predictedDistance < bestDistance) ||
        (score == bestScore && predictedDistance == bestDistance && effectiveConfidence > bestConfidence) ||
        (score == bestScore && predictedDistance == bestDistance && effectiveConfidence == bestConfidence && ageMs < bestAge)) {
      bestIndex = i;
      bestDistance = predictedDistance;
      bestConfidence = effectiveConfidence;
      bestScore = score;
      bestAge = ageMs;
    }
  }

  activeVisibleTargets = visibleTargets;

  if (bestIndex >= 0) {
    primaryTargetBin = bestIndex;
    primaryTargetAngle = predictedAngleForBin(bestIndex, nowMs);
    primaryTargetDistance = predictedDistanceForBin(bestIndex, nowMs);
    primaryTargetConfidence = bestConfidence;
    primaryTargetScore = bestScore;
    primaryTargetAngularVelocity = targetBinAngularVelocity[bestIndex];
    primaryTargetRadialVelocity = targetBinRadialVelocity[bestIndex];
    primaryTargetHits = targetBinHits[bestIndex];
    currentState = stateFromDistance(primaryTargetDistance);
    primaryTargetTrackable =
      currentState == DANGER_TRACK &&
      primaryTargetConfidence >= TARGET_CONFIDENCE_MIN_TRACK &&
      primaryTargetHits >= TARGET_HITS_MIN_TRACK &&
      isAimableRadarAngle(primaryTargetAngle);
  } else {
    resetTargetSelection();
  }
}

void updateDistanceReading() {
#if DIAG_DISABLE_SENSOR
  realDistance = -1;
  selectPrimaryTarget();
  serialPacketPending = true;
  return;
#endif

  unsigned long nowMs = millis();
  if (nowMs - lastSensorMs < SENSOR_MEASURE_INTERVAL_MS) return;
  lastSensorMs = nowMs;

  if (!radarAngleKnown) {
    lastMeasurementAngle = currentStepperAngle;
    realDistance = -1;
    selectPrimaryTarget();
    serialPacketPending = true;
    return;
  }

  lastMeasurementAngle = currentStepperAngle;
  long duration = readEchoDuration();

  if (duration == 0) {
    realDistance = -1;
    selectPrimaryTarget();
    serialPacketPending = true;
    return;
  }

  int distance = (int)(duration * 0.0343f * 0.5f + 0.5f);
  if (distance > TARGET_DETECT_DISTANCE) {
    realDistance = TARGET_DETECT_DISTANCE + 1;
    selectPrimaryTarget();
    serialPacketPending = true;
    return;
  }

  if (isBlindDistance(distance)) {
    realDistance = 0;
    selectPrimaryTarget();
    serialPacketPending = true;
    return;
  }

  realDistance = distance;
  updateTargetMap(lastMeasurementAngle, distance);
  selectPrimaryTarget();
  serialPacketPending = true;
}

int calculateSmartElevation(int distance, int radialVelocity) {
  if (distance <= BLIND_ZONE_MAX_DISTANCE || distance > DANGER_MAX_DISTANCE) {
    return DEFAULT_ELEVATION;
  }

  // This hardware scans horizontally only, so elevation is estimated from predicted range and motion.
  float clampedDistance = (float)constrain(distance, BLIND_ZONE_MAX_DISTANCE + 1, DANGER_MAX_DISTANCE);
  float closeness = (DANGER_MAX_DISTANCE - clampedDistance) / (float)(DANGER_MAX_DISTANCE - BLIND_ZONE_MAX_DISTANCE);
  if (closeness < 0.0f) closeness = 0.0f;
  if (closeness > 1.0f) closeness = 1.0f;

  float closeLift = ELEVATION_CLOSE_LIFT_DEG * closeness * closeness;
  float geometryLift = atan2(TARGET_REFERENCE_HEIGHT_CM - LAUNCH_REFERENCE_HEIGHT_CM, clampedDistance + ELEVATION_OFFSET_CM) * DEG_PER_RAD;
  float distanceMeters = clampedDistance / 100.0f;
  float dropLift = distanceMeters * distanceMeters * ELEVATION_DROP_GAIN_DEG_PER_M2;
  float approachLift = 0.0f;

  if (radialVelocity < 0) {
    approachLift = (-radialVelocity) * ELEVATION_APPROACH_GAIN_DEG_PER_CMPS;
    if (approachLift > 3.5f) approachLift = 3.5f;
  }

  float rawAngle = DEFAULT_ELEVATION + ELEVATION_PLATFORM_BIAS_DEG + closeLift + geometryLift + dropLift + approachLift;
  return constrain((int)(rawAngle + 0.5f), MIN_ELEVATION, MAX_ELEVATION);
}

float calculateSmartElevationFromLauncherVector(float forwardCm, float rightCm, float upCm, int radialVelocity) {
  float horizontalDistanceCm = sqrt(forwardCm * forwardCm + rightCm * rightCm);
  if (horizontalDistanceCm < 1.0f) horizontalDistanceCm = 1.0f;

  float clampedDistance = constrain(horizontalDistanceCm, (float)(BLIND_ZONE_MAX_DISTANCE + 1), (float)DANGER_MAX_DISTANCE);
  float closeness = (DANGER_MAX_DISTANCE - clampedDistance) / (float)(DANGER_MAX_DISTANCE - BLIND_ZONE_MAX_DISTANCE);
  if (closeness < 0.0f) closeness = 0.0f;
  if (closeness > 1.0f) closeness = 1.0f;

  float pitchOffsetDeg = atan2(upCm, horizontalDistanceCm + ELEVATION_OFFSET_CM) * DEG_PER_RAD;
  float closeLift = ELEVATION_CLOSE_LIFT_DEG * closeness * closeness;
  float distanceMeters = horizontalDistanceCm / 100.0f;
  float dropLift = distanceMeters * distanceMeters * ELEVATION_DROP_GAIN_DEG_PER_M2;
  float approachLift = 0.0f;

  if (radialVelocity < 0) {
    approachLift = (-radialVelocity) * ELEVATION_APPROACH_GAIN_DEG_PER_CMPS;
    if (approachLift > 3.5f) approachLift = 3.5f;
  }

  float rawAngle = DEFAULT_ELEVATION + ELEVATION_PLATFORM_BIAS_DEG + pitchOffsetDeg + closeLift + dropLift + approachLift;
  return constrainFloatValue(rawAngle, (float)MIN_ELEVATION, (float)MAX_ELEVATION);
}

void updateAimTargets() {
  unsigned long nowMs = millis();

  if (!radarAngleKnown) {
    lastTrackableAimMs = 0;
    desiredAzimuthAngle = HOME_AZIMUTH;
    desiredElevationAngle = DEFAULT_ELEVATION;
    targetAzimuthAngle = HOME_AZIMUTH;
    targetElevationAngle = DEFAULT_ELEVATION;
    resetAimStabilizer();
    return;
  }

  if (!primaryTargetTrackable) {
    unsigned long timeSinceLastTargetMs = lastTrackableAimMs == 0 ? AIM_HOME_RETURN_DELAY_MS : nowMs - lastTrackableAimMs;
    if (lastTrackableAimMs != 0 && timeSinceLastTargetMs < AIM_HOME_RETURN_DELAY_MS) {
      float holdElevation = timeSinceLastTargetMs < ELEVATION_HOME_RETURN_DELAY_MS ? (float)TARGETING_ELEVATION : (float)DEFAULT_ELEVATION;
      desiredElevationAngle = constrainFloatValue(holdElevation, (float)MIN_ELEVATION, (float)MAX_ELEVATION);
      targetElevationCommand = desiredElevationAngle;
      targetElevationAngle = constrain(roundFloatToInt(targetElevationCommand), MIN_ELEVATION, MAX_ELEVATION);
      pauseAimStabilizer();
      return;
    }

    lastTrackableAimMs = 0;
    desiredAzimuthAngle = HOME_AZIMUTH;
    desiredElevationAngle = DEFAULT_ELEVATION;
    targetAzimuthAngle = HOME_AZIMUTH;
    targetElevationAngle = DEFAULT_ELEVATION;
    resetAimStabilizer();
    return;
  }

  lastTrackableAimMs = nowMs;

  float rawForwardCm;
  float rawRightCm;
  float rawUpCm;
  radarAngleDistanceToLauncherVector(primaryTargetAngle, primaryTargetDistance, &rawForwardCm, &rawRightCm, &rawUpCm);

  float dtSec = SENSOR_MEASURE_INTERVAL_MS / 1000.0f;
  if (!aimFilterInitialized) {
    filteredAimForwardCm = rawForwardCm;
    filteredAimRightCm = rawRightCm;
    filteredAimUpCm = rawUpCm;
    previousAimForwardCm = filteredAimForwardCm;
    previousAimRightCm = filteredAimRightCm;
    previousAimUpCm = filteredAimUpCm;
    targetAzimuthCommand = launchAzimuthPosition;
    targetElevationCommand = launchElevationPosition;
    aimFilterInitialized = true;
  } else {
    if (lastAimFilterMs != 0) {
      dtSec = (nowMs - lastAimFilterMs) / 1000.0f;
      if (dtSec < 0.02f) dtSec = 0.02f;
      if (dtSec > 0.50f) dtSec = 0.50f;
    }

    previousAimForwardCm = filteredAimForwardCm;
    previousAimRightCm = filteredAimRightCm;
    previousAimUpCm = filteredAimUpCm;
    filteredAimForwardCm = smoothFloatValue(filteredAimForwardCm, rawForwardCm, AIM_SMOOTHING_ALPHA);
    filteredAimRightCm = smoothFloatValue(filteredAimRightCm, rawRightCm, AIM_SMOOTHING_ALPHA);
    filteredAimUpCm = smoothFloatValue(filteredAimUpCm, rawUpCm, AIM_SMOOTHING_ALPHA);
  }
  lastAimFilterMs = nowMs;

  float vx = (filteredAimForwardCm - previousAimForwardCm) / dtSec;
  float vy = (filteredAimRightCm - previousAimRightCm) / dtSec;
  float predictSec = AIM_PREDICTION_MS / 1000.0f;
  float predictedForwardCm = filteredAimForwardCm + vx * predictSec;
  float predictedRightCm = filteredAimRightCm + vy * predictSec;

  float desiredRadarAngle = launcherVectorToRadarAngleFloat(predictedForwardCm, predictedRightCm);
  desiredAzimuthAngle = constrainFloatValue(
    radarAngleToAzimuthFloat(desiredRadarAngle) + AZIMUTH_FIRE_CORRECTION_DEG,
    (float)MIN_AZIMUTH,
    (float)MAX_AZIMUTH
  );
  desiredElevationAngle = constrainFloatValue((float)TARGETING_ELEVATION, (float)MIN_ELEVATION, (float)MAX_ELEVATION);

  if (fabs(desiredAzimuthAngle - targetAzimuthCommand) > AZIMUTH_COMMAND_DEADBAND_DEG) {
    targetAzimuthCommand = dampFloatValue(targetAzimuthCommand, desiredAzimuthAngle, AIM_DAMPING);
    if (fabs(targetAzimuthCommand - desiredAzimuthAngle) < AZIMUTH_COMMAND_DEADBAND_DEG) targetAzimuthCommand = desiredAzimuthAngle;
  } else {
    targetAzimuthCommand = desiredAzimuthAngle;
  }
  targetAzimuthCommand = constrainFloatValue(targetAzimuthCommand, (float)MIN_AZIMUTH, (float)MAX_AZIMUTH);
  targetAzimuthAngle = constrain(roundFloatToInt(targetAzimuthCommand), MIN_AZIMUTH, MAX_AZIMUTH);

  if (fabs(desiredElevationAngle - targetElevationCommand) > ELEVATION_COMMAND_DEADBAND_DEG) {
    targetElevationCommand = dampFloatValue(targetElevationCommand, desiredElevationAngle, AIM_DAMPING);
    if (fabs(targetElevationCommand - desiredElevationAngle) < ELEVATION_COMMAND_DEADBAND_DEG) targetElevationCommand = desiredElevationAngle;
  } else {
    targetElevationCommand = desiredElevationAngle;
  }
  targetElevationCommand = constrainFloatValue(targetElevationCommand, (float)MIN_ELEVATION, (float)MAX_ELEVATION);
  targetElevationAngle = constrain(roundFloatToInt(targetElevationCommand), MIN_ELEVATION, MAX_ELEVATION);
}

void serviceServos() {
  unsigned long nowMs = millis();
  if (nowMs - lastServoMs < SERVO_UPDATE_INTERVAL_MS) return;
  lastServoMs = nowMs;

  float azimuthError = targetAzimuthCommand - launchAzimuthPosition;
  float azimuthMagnitude = fabs(azimuthError);
  if (azimuthMagnitude > SERVO_POSITION_DEADBAND_DEG) {
    float step = (float)dynamicServoStep(roundFloatToInt(azimuthMagnitude), SERVO_AZIMUTH_STEP_DEG, SERVO_AZIMUTH_FAST_STEP_DEG);
    if (step > azimuthMagnitude) step = azimuthMagnitude;
    launchAzimuthPosition += azimuthError > 0.0f ? step : -step;
  } else if (azimuthMagnitude > 0.01f) {
    launchAzimuthPosition = targetAzimuthCommand;
  }
  launchAzimuthPosition = constrainFloatValue(launchAzimuthPosition, (float)MIN_AZIMUTH, (float)MAX_AZIMUTH);
  launchAzimuthAngle = constrain(roundFloatToInt(launchAzimuthPosition), MIN_AZIMUTH, MAX_AZIMUTH);
  writeLaunchAzimuthServo(launchAzimuthPosition);

  float elevationError = targetElevationCommand - launchElevationPosition;
  float elevationMagnitude = fabs(elevationError);
  if (elevationMagnitude > SERVO_POSITION_DEADBAND_DEG) {
    float step = (float)dynamicServoStep(roundFloatToInt(elevationMagnitude), SERVO_ELEVATION_STEP_DEG, SERVO_ELEVATION_FAST_STEP_DEG);
    if (step > elevationMagnitude) step = elevationMagnitude;
    launchElevationPosition += elevationError > 0.0f ? step : -step;
  } else if (elevationMagnitude > 0.01f) {
    launchElevationPosition = targetElevationCommand;
  }
  launchElevationPosition = constrainFloatValue(launchElevationPosition, (float)MIN_ELEVATION, (float)MAX_ELEVATION);
  launchElevationAngle = constrain(roundFloatToInt(launchElevationPosition), MIN_ELEVATION, MAX_ELEVATION);
  writeLaunchElevationServo(launchElevationPosition);
}

void playAlertPattern(unsigned long nowMs, unsigned long intervalMs, unsigned int freqA, unsigned int freqB, unsigned int durationMs) {
  if (lastAlertToneMs != 0 && nowMs - lastAlertToneMs < intervalMs) return;

  lastAlertToneMs = nowMs;
  unsigned int frequency = (((nowMs / intervalMs) % 2UL) == 0UL) ? freqA : freqB;
  tone(BUZZER_PIN, frequency, durationMs);
}

void updateAlerts() {
  unsigned long nowMs = millis();

  if (currentState != lastAlertState) {
    lastAlertState = currentState;
    lastAlertToneMs = 0;
    noTone(BUZZER_PIN);
  }

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  switch (currentState) {
    case DANGER_TRACK:
      digitalWrite(RED_LED_PIN, HIGH);
      playAlertPattern(nowMs, 220UL, 3200, 3800, 120);
      break;

    case MID_WARN:
      digitalWrite(YELLOW_LED_PIN, HIGH);
      playAlertPattern(nowMs, 650UL, 2300, 2550, 95);
      break;

    case FAR_WARN:
      digitalWrite(GREEN_LED_PIN, HIGH);
      playAlertPattern(nowMs, 1200UL, 1650, 1800, 70);
      break;

    default:
      noTone(BUZZER_PIN);
      break;
  }
}

void sendSerialPacket() {
  unsigned long nowMs = millis();
  if (!serialPacketPending) return;
  if (nowMs - lastSerialMs < SERIAL_UPDATE_INTERVAL_MS) return;
  lastSerialMs = nowMs;
  serialPacketPending = false;

  Serial.print(currentStepperAngle);
  Serial.print(',');
  Serial.print(realDistance);
  Serial.print(',');
  Serial.print((int)currentState);
  Serial.print(',');
  Serial.print(launchAzimuthAngle);
  Serial.print(',');
  Serial.println(launchElevationAngle);
}

void setup() {
  startupResetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RADAR_HOME_SENSOR_PIN, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(stepperPins[i], OUTPUT);
  }

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(TRIG_PIN, LOW);
  stopFire();
  noTone(BUZZER_PIN);
  stepperOff();
  resetTrackingState();

  launchAzimuth.attach(LAUNCH_AZIMUTH_PIN);
  launchElevation.attach(LAUNCH_ELEVATION_PIN);
  launchAzimuthPosition = (float)launchAzimuthAngle;
  launchElevationPosition = (float)launchElevationAngle;
  writeLaunchAzimuthServo(launchAzimuthPosition);
  writeLaunchElevationServo(launchElevationPosition);

  tone(BUZZER_PIN, 2200, 60);
  digitalWrite(RED_LED_PIN, HIGH);
  delay(120);
  tone(BUZZER_PIN, 2800, 60);
  digitalWrite(RED_LED_PIN, LOW);

  lastStepperStepUs = micros();
  initializeRadarReference();
  printResetCause();
  printSerialHelp();
  Serial.print(F("# HOME SENSOR: 49E on A5, detect >= "));
  Serial.print(HOME_SENSOR_DETECT_THRESHOLD);
  Serial.print(F(", release <= "));
  Serial.println(HOME_SENSOR_RELEASE_THRESHOLD);
  printRadarStatus();

#if DIAG_DISABLE_SENSOR
  Serial.println(F("# DIAG: SENSOR DISABLED"));
#endif

  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  readSerialCommands();
  serviceHomeSensor();
  serviceStepper();
  updateDistanceReading();
  serviceStepper();
  updateAimTargets();
  serviceServos();
  serviceFireControl();
  updateAlerts();
  serviceStepper();
  sendSerialPacket();
  wdt_reset();
}
