/*
Smart turret radar display.
Serial packet format: angle,distance,state,azimuth,elevation
*/

import processing.serial.*;

EnhancedRadarSystem radar;

void setup() {
  size(1600, 900);
  smooth();
  frameRate(45);

  radar = new EnhancedRadarSystem(this);
  radar.initialize();
  println("تم تشغيل نظام الرادار");
}

void draw() {
  try {
    radar.update();
  } catch (Exception e) {
    println("Draw loop recovered from exception: " + e.getMessage());
  }
}

void serialEvent(Serial myPort) {
  // Serial is read by pollSerialPort() in draw(); mixing both paths can stall Processing serial on Windows.
}

void keyPressed() {
  if (radar != null) {
    radar.handleKeyPress();
  }
}

void keyReleased() {
  if (radar != null) {
    radar.handleKeyRelease();
  }
}

class RadarPing {
  float angle;
  float distance;
  long timestamp;

  RadarPing(float angle, float distance) {
    this.angle = angle;
    this.distance = distance;
    this.timestamp = millis();
  }
}

class EnhancedRadarSystem {
  PApplet p;
  Serial myPort;

  String preferredPortHint = "COM";
  String portName = "";
  String connectionStatus = "غير متصل";
  boolean isConnected = false;
  int baudRate = 115200;
  long lastSerialDataMs = 0;
  long lastPortOpenMs = 0;

  final int minAzimuth = 0;
  final int maxAzimuth = 170;
  final int homeAzimuth = 60;
  final boolean azimuthDirectionInverted = true;
  final int radarForwardAngle = 0;
  final int turretForwardRadarOffsetDeg = 0;
  final int radarAimLeftSpan = 110;
  final int radarAimRightSpan = 90;

  PFont font;
  PFont largeFont;
  PFont extraLargeFont;

  PVector radarCenter;
  float radarRadius;

  final int displayMaxRangeCm = 500;
  final int nearPingLifetimeMs = 1200;
  final int farPingLifetimeMs = 3200;
  final int maxSerialLinesPerFrame = 32;
  final int maxQueuedSerialLines = 80;
  final int maxQueuedSerialCommands = 16;
  final int maxSerialPollCharsPerFrame = 2048;
  final int serialCommandIntervalMs = 20;
  final int manualMoveRepeatMs = 20;
  final int serialSoftRecoverMs = 2500;
  final int serialHardRecoverMs = 7000;

  ArrayList<RadarPing> pings = new ArrayList<RadarPing>();
  ArrayList<String> pendingSerialLines = new ArrayList<String>();
  ArrayList<String> pendingSerialCommands = new ArrayList<String>();
  String incomingSerialBuffer = "";

  int iAngle = 0;
  int iDistance = -1;
  int iState = 0;
  int iAzimuth = homeAzimuth;
  int iElevation = 45;
  int engagedTargetCount = 0;

  float currentAngle = 0;
  float currentDistance = 0;
  float animatedTurretAzimuth = homeAzimuth;
  float animatedTurretElevation = 45;
  float lastPlottedAngle = -1000;
  float lastPlottedDistance = -1000;
  long lastPlottedTime = 0;
  long manualFireFlashUntilMs = 0;
  long lastSerialCommandMs = 0;
  long lastManualMoveMs = 0;
  long lastSerialRecoveryMs = 0;
  long lastRawSerialLineMs = 0;
  long lastRadarPacketMs = 0;

  String alarmStatusText = "آمن";
  String threatLevelText = "دورية بحث روتينية";
  String lastTargetClockText = "--:--:--";
  String lastTargetDateText = "--/--/----";
  String lastRawSerialLine = "---";
  String controlStatusText = "AUTO";
  boolean autoControlEnabled = true;
  boolean manualLeftHeld = false;
  boolean manualRightHeld = false;
  boolean manualUpHeld = false;
  boolean manualDownHeld = false;
  boolean targetDetected = false;
  int totalDetectedTargets = 0;

  EnhancedRadarSystem(PApplet p) {
    this.p = p;
    radarCenter = new PVector(p.width * 0.50f, p.height * 0.485f);
    radarRadius = min(p.width * 0.275f, p.height * 0.395f);
  }

  void initialize() {
    font = p.createFont("Tahoma", 14);
    largeFont = p.createFont("Tahoma", 18);
    extraLargeFont = p.createFont("Tahoma Bold", 24);

    println("Available serial ports:");
    printAvailablePorts();
    setupSerial();
    p.background(10, 15, 20);
  }

  void update() {
    pollSerialPort();
    serviceSerialInput();
    serviceOutgoingSerialCommands();
    serviceSerialRecovery();
    serviceManualMoveHold();
    updateWeaponAnimation();
    drawHudBackdrop();
    cleanupOldPings();
    drawHeader();
    drawMainRadar();
    drawLeftInfoPanel();
    drawRightInfoPanel();
    drawStatusBar();
    handleVisualAlarm();
  }

  void updateWeaponAnimation() {
    animatedTurretAzimuth += (iAzimuth - animatedTurretAzimuth) * 0.12f;
    animatedTurretElevation += (iElevation - animatedTurretElevation) * 0.14f;
  }

  void printAvailablePorts() {
    String[] ports = Serial.list();
    if (ports == null || ports.length == 0) {
      println("  (none)");
      return;
    }
    for (int i = 0; i < ports.length; i++) {
      println("  [" + i + "] " + ports[i]);
    }
  }

  String normalizePortName(String value) {
    if (value == null) return "";
    return value.toLowerCase().replace("\\\\.\\", "");
  }

  String pickBestPort() {
    String[] ports = Serial.list();
    if (ports == null || ports.length == 0) return "";

    String hint = normalizePortName(preferredPortHint);
    for (int i = 0; i < ports.length; i++) {
      String normalized = normalizePortName(ports[i]);
      if (normalized.equals(hint) || normalized.endsWith(hint)) {
        return ports[i];
      }
    }
    for (int i = 0; i < ports.length; i++) {
      if (normalizePortName(ports[i]).indexOf(hint) >= 0) {
        return ports[i];
      }
    }
    return ports[0];
  }

  void closeSerial() {
    synchronized (pendingSerialLines) {
      pendingSerialLines.clear();
    }
    synchronized (pendingSerialCommands) {
      pendingSerialCommands.clear();
    }
    incomingSerialBuffer = "";
    lastRawSerialLineMs = 0;
    lastRadarPacketMs = 0;
    lastRawSerialLine = "---";

    if (myPort != null) {
      try {
        myPort.stop();
      } catch (Exception e) {
      }
      myPort = null;
      p.delay(120);
    }
  }

  void setupSerial() {
    closeSerial();
    portName = pickBestPort();

    if (portName == null || portName.length() == 0) {
      isConnected = false;
      connectionStatus = "لا يوجد منفذ Serial";
      return;
    }

    try {
      myPort = new Serial(p, portName, baudRate);
      p.delay(1200);
      myPort.clear();
      isConnected = true;
      lastSerialDataMs = 0;
      lastSerialRecoveryMs = 0;
      lastRawSerialLineMs = 0;
      lastRadarPacketMs = 0;
      lastRawSerialLine = "---";
      lastPortOpenMs = p.millis();
      connectionStatus = "تم فتح " + portName + " وجار انتظار البيانات";
      println("Connected to serial port: " + portName);
    } catch (Exception e) {
      isConnected = false;
      connectionStatus = "خطأ في الاتصال عبر " + portName;
      println("Failed to connect to serial port: " + portName);
    }
  }

  void serviceSerialInput() {
    try {
      int linesProcessed = 0;
      while (linesProcessed < maxSerialLinesPerFrame) {
        String data = null;
        synchronized (pendingSerialLines) {
          if (pendingSerialLines.size() == 0) break;
          data = pendingSerialLines.remove(0);
        }
        processSerialData(data.trim());
        lastSerialDataMs = p.millis();
        connectionStatus = "متصل عبر " + portName;
        linesProcessed++;
      }
    } catch (Exception e) {
      isConnected = false;
      connectionStatus = "انقطع الاتصال: " + portName;
      closeSerial();
      println("Serial polling error: " + e.getMessage());
    }
  }

  void enqueueSerialLine(String data) {
    if (data == null) return;
    data = data.trim();
    if (data.length() == 0) return;

    lastRawSerialLineMs = p.millis();
    lastRawSerialLine = data;

    synchronized (pendingSerialLines) {
      pendingSerialLines.add(data);
      while (pendingSerialLines.size() > maxQueuedSerialLines) {
        pendingSerialLines.remove(0);
      }
    }
  }

  void pollSerialPort() {
    if (myPort == null || !isConnected) return;

    try {
      int charsRead = 0;
      while (myPort.available() > 0 && charsRead < maxSerialPollCharsPerFrame) {
        char c = (char)myPort.read();
        charsRead++;

        if (c == '\r') continue;
        if (c == '\n') {
          enqueueSerialLine(incomingSerialBuffer);
          incomingSerialBuffer = "";
          continue;
        }

        if (incomingSerialBuffer.length() < 120) {
          incomingSerialBuffer += c;
        } else {
          incomingSerialBuffer = "";
        }
      }
    } catch (Exception e) {
      isConnected = false;
      connectionStatus = "ط§ظ†ظ‚ط·ط¹ ط§ظ„ط§طھطµط§ظ„: " + portName;
      closeSerial();
      println("Serial poll error: " + e.getMessage());
    }
  }

  void serviceSerialRecovery() {
    if (myPort == null || !isConnected) return;
    if (lastRawSerialLineMs == 0) return;

    long now = p.millis();
    long ageMs = now - lastRawSerialLineMs;
    if (ageMs < serialSoftRecoverMs) return;
    if (now - lastSerialRecoveryMs < serialSoftRecoverMs) return;

    lastSerialRecoveryMs = now;
    incomingSerialBuffer = "";

    if (ageMs < serialHardRecoverMs) {
      try {
        myPort.clear();
        controlStatusText = "SERIAL CLEAR";
      } catch (Exception e) {
        isConnected = false;
        connectionStatus = "ط§ظ†ظ‚ط·ط¹ ط§ظ„ط§طھطµط§ظ„: " + portName;
        closeSerial();
      }
      return;
    }

    controlStatusText = "SERIAL REOPEN";
    setupSerial();
  }

  void queueSerialEvent(Serial port) {
    return;
/*
    } catch (Exception e) {
      isConnected = false;
      connectionStatus = "انقطع الاتصال: " + portName;
      closeSerial();
      println("Serial event error: " + e.getMessage());
    }
*/
  }

  boolean queueSerialCommand(String command) {
    if (myPort == null || !isConnected) {
      controlStatusText = "NO SERIAL";
      return false;
    }

    synchronized (pendingSerialCommands) {
      pendingSerialCommands.add(command);
      while (pendingSerialCommands.size() > maxQueuedSerialCommands) {
        pendingSerialCommands.remove(0);
      }
    }
    controlStatusText = command.toUpperCase();
    if (command.equals("fire")) {
      manualFireFlashUntilMs = p.millis() + 650;
    }
    return true;
  }

  void serviceOutgoingSerialCommands() {
    if (myPort == null || !isConnected) return;
    if (p.millis() - lastSerialCommandMs < serialCommandIntervalMs) return;

    String command = null;
    synchronized (pendingSerialCommands) {
      if (pendingSerialCommands.size() == 0) return;
      command = pendingSerialCommands.remove(0);
    }

    try {
      myPort.write(command + "\n");
      lastSerialCommandMs = p.millis();
      lastSerialDataMs = p.millis();
    } catch (Exception e) {
      isConnected = false;
      connectionStatus = "انقطع الاتصال: " + portName;
      controlStatusText = "SEND ERROR";
      closeSerial();
    }
  }

  void handleKeyPress() {
    if (key == PConstants.CODED) {
      if (!autoControlEnabled) handleArrowKeyState(keyCode, true);
      return;
    }
    if (key == 'r' || key == 'R' || key == 'ق') {
      setupSerial();
    } else if (key == 'p' || key == 'P' || key == 'ح') {
      printAvailablePorts();
    } else if (key >= '0' && key <= '9') {
      preferredPortHint = "COM" + key;
      setupSerial();
      println("Preferred port changed to: " + preferredPortHint);
    } else if (key == 'a' || key == 'A' || key == 'ش') {
      boolean nextAutoControlEnabled = !autoControlEnabled;
      if (queueSerialCommand(nextAutoControlEnabled ? "auto on" : "auto off")) {
        autoControlEnabled = nextAutoControlEnabled;
        clearManualMoveKeys();
      }
    } else if (key == 'f' || key == 'F' || key == 'ب') {
      queueSerialCommand("fire");
    } else if (key == 's' || key == 'S' || key == 'س') {
      queueSerialCommand("cease");
    }
  }

  void handleKeyRelease() {
    if (key == PConstants.CODED) {
      handleArrowKeyState(keyCode, false);
    }
  }

  void clearManualMoveKeys() {
    manualLeftHeld = false;
    manualRightHeld = false;
    manualUpHeld = false;
    manualDownHeld = false;
  }

  void handleArrowKeyState(int code, boolean pressed) {
    if (code == PConstants.LEFT) {
      boolean wasHeld = manualLeftHeld;
      manualLeftHeld = pressed;
      if (pressed && !wasHeld) sendManualMoveStep();
    } else if (code == PConstants.RIGHT) {
      boolean wasHeld = manualRightHeld;
      manualRightHeld = pressed;
      if (pressed && !wasHeld) sendManualMoveStep();
    } else if (code == PConstants.UP) {
      boolean wasHeld = manualUpHeld;
      manualUpHeld = pressed;
      if (pressed && !wasHeld) sendManualMoveStep();
    } else if (code == PConstants.DOWN) {
      boolean wasHeld = manualDownHeld;
      manualDownHeld = pressed;
      if (pressed && !wasHeld) sendManualMoveStep();
    }
  }

  void serviceManualMoveHold() {
    if (autoControlEnabled) return;
    if (!manualLeftHeld && !manualRightHeld && !manualUpHeld && !manualDownHeld) return;
    if (p.millis() - lastManualMoveMs < manualMoveRepeatMs) return;
    sendManualMoveStep();
  }

  void sendManualMoveStep() {
    if (autoControlEnabled) return;

    int azimuthStep = 0;
    int elevationStep = 0;
    if (manualLeftHeld) azimuthStep += 1;
    if (manualRightHeld) azimuthStep -= 1;
    if (manualUpHeld) elevationStep += 1;
    if (manualDownHeld) elevationStep -= 1;
    if (azimuthStep == 0 && elevationStep == 0) return;

    if (queueSerialCommand("move " + azimuthStep + " " + elevationStep)) {
      lastManualMoveMs = p.millis();
    }
  }

  void processSerialData(String data) {
    if (data == null || data.length() == 0 || data.startsWith("#")) return;

    String[] parts = data.split(",");
    if (parts.length < 5) return;

    try {
      int rawAngle = p.parseInt(parts[0]);
      iAngle = wrapAngle360(rawAngle);
      iDistance = p.parseInt(parts[1]);
      iState = p.parseInt(parts[2]);
      iAzimuth = p.parseInt(parts[3]);
      iElevation = p.parseInt(parts[4]);
      if (parts.length >= 6) {
        engagedTargetCount = max(0, p.parseInt(parts[5]));
      }
      lastRadarPacketMs = p.millis();

      float delta = angleDelta(currentAngle, iAngle);
      currentAngle = wrapAngle360((int)(currentAngle + delta * 0.45f));

      if (isValidDistance(iDistance)) {
        currentDistance = p.lerp(currentDistance, iDistance, 0.3f);
        addPingIfNeeded(iAngle, iDistance);
      } else {
        currentDistance = p.lerp(currentDistance, 0, 0.15f);
      }

      updateStateTexts();
    } catch (Exception e) {
    }
  }

  void addPingIfNeeded(int angle, int distance) {
    long now = p.millis();
    float angleDiff = abs(angleDelta(lastPlottedAngle, angle));
    float distanceDiff = abs(distance - lastPlottedDistance);

    if (lastPlottedAngle < -360 || angleDiff >= 5 || distanceDiff >= 10 || now - lastPlottedTime >= 250) {
      synchronized (pings) {
        RadarPing existing = findNearbyPing(angle, distance);
        if (existing != null) {
          existing.angle = wrapAngle360((int)(existing.angle + angleDelta(existing.angle, angle) * 0.5f));
          existing.distance = p.lerp(existing.distance, distance, 0.5f);
          existing.timestamp = now;
        } else {
          pings.add(new RadarPing(angle, distance));
        }
      }
      lastPlottedAngle = angle;
      lastPlottedDistance = distance;
      lastPlottedTime = now;
      if (!targetDetected && isValidDistance(distance)) recordLastTargetTime();
      targetDetected = isValidDistance(distance);
    }
  }

  RadarPing findNearbyPing(int angle, int distance) {
    float maxAngleDiff = distance > 250 ? 8 : 5;
    float maxDistanceDiff = distance > 250 ? 35 : 18;

    for (int i = pings.size() - 1; i >= 0; i--) {
      RadarPing ping = pings.get(i);
      if (abs(angleDelta(ping.angle, angle)) <= maxAngleDiff &&
          abs(ping.distance - distance) <= maxDistanceDiff) {
        return ping;
      }
    }
    return null;
  }

  int pingLifetimeMs(float distance) {
    if (distance > 250) return farPingLifetimeMs;
    return nearPingLifetimeMs;
  }

  void cleanupOldPings() {
    long now = p.millis();
    int count = 0;
    synchronized (pings) {
      while (pings.size() > 140) {
        pings.remove(0);
      }
      for (int i = pings.size() - 1; i >= 0; i--) {
        RadarPing ping = pings.get(i);
        if (now - ping.timestamp > pingLifetimeMs(ping.distance)) {
          pings.remove(i);
        } else if (isValidDistance((int)ping.distance)) {
          count++;
        }
      }
    }
    totalDetectedTargets = count;
  }

  void drawHudBackdrop() {
    p.background(2, 8, 8);

    p.strokeWeight(1);
    for (int x = 0; x <= p.width; x += 40) {
      p.stroke(20, 90, 70, x % 160 == 0 ? 55 : 25);
      p.line(x, 0, x, p.height);
    }
    for (int y = 0; y <= p.height; y += 40) {
      p.stroke(20, 90, 70, y % 160 == 0 ? 55 : 25);
      p.line(0, y, p.width, y);
    }

    p.noStroke();
    for (int y = 0; y < p.height; y += 4) {
      p.fill(0, 0, 0, 22);
      p.rect(0, y, p.width, 1);
    }

    p.fill(0, 0, 0, 120);
    p.rect(0, 0, p.width, 78);
    p.rect(0, p.height - 92, p.width, 92);
  }

  color hudBlue() {
    return p.color(110, 190, 255);
  }

  color hudGreen() {
    return p.color(65, 255, 105);
  }

  color radarOrange() {
    return p.color(230, 95, 12);
  }

  color radarOrangeStrong() {
    return p.color(255, 115, 18);
  }

  color alertColor() {
    if (iState == 3) return p.color(255, 70, 50);
    if (iState == 2) return p.color(255, 220, 70);
    if (iState == 1) return p.color(80, 255, 130);
    return hudBlue();
  }

  String modeLabel() {
    if (iState == 3) return "TRACK";
    if (iState == 2) return "MID";
    if (iState == 1) return "SCAN";
    return "IDLE";
  }

  String modeArabicLabel() {
    if (iState == 3) return "تتبع";
    if (iState == 2) return "متوسط";
    if (iState == 1) return "مسح";
    return "خمول";
  }

  String fireLabel() {
    if (iState == 3) return "READY";
    return "SAFE";
  }

  String fireArabicLabel() {
    if (iState == 3) return "جاهز";
    return "آمن";
  }

  String autoFireArabicLabel() {
    if (autoControlEnabled) return "تلقائي";
    return "يدوي";
  }

  String connectionArabicLabel() {
    if (isConnected) return "نشط";
    return "غير متصل";
  }

  long radarPacketAgeMs() {
    if (lastRadarPacketMs == 0) return -1;
    return p.millis() - lastRadarPacketMs;
  }

  long rawSerialAgeMs() {
    if (lastRawSerialLineMs == 0) return -1;
    return p.millis() - lastRawSerialLineMs;
  }

  String formatAgeLabel(String prefix, long ageMs) {
    if (ageMs < 0) return prefix + ": WAIT";
    if (ageMs < 1000) return prefix + ": " + ageMs + " ms";
    return prefix + ": " + p.nf(ageMs / 1000.0f, 0, 1) + " s";
  }

  String radarPacketLabel() {
    return formatAgeLabel("DATA", radarPacketAgeMs());
  }

  String rawSerialLabel() {
    return formatAgeLabel("RAW", rawSerialAgeMs());
  }

  color radarPacketColor() {
    return ageColor(radarPacketAgeMs());
  }

  color rawSerialColor() {
    return ageColor(rawSerialAgeMs());
  }

  color ageColor(long ageMs) {
    if (ageMs < 0) return p.color(255, 210, 70);
    if (ageMs <= 350) return hudGreen();
    if (ageMs <= 1200) return p.color(255, 210, 70);
    return p.color(255, 70, 55);
  }

  String shortRawSerialLine() {
    if (lastRawSerialLine == null || lastRawSerialLine.length() == 0) return "LAST: ---";
    String value = lastRawSerialLine;
    if (value.length() > 28) value = value.substring(0, 28) + "...";
    return "LAST: " + value;
  }

  void hudPanel(float x, float y, float w, float h, String title) {
    p.pushStyle();
    p.stroke(hudBlue());
    p.strokeWeight(1.5f);
    p.fill(5, 16, 16, 218);
    p.rect(x, y, w, h, 8);

    p.stroke(35, 255, 120, 80);
    p.line(x + 14, y + 12, x + w - 14, y + 12);
    p.line(x + 14, y + h - 12, x + w - 14, y + h - 12);

    p.textAlign(PConstants.CENTER);
    p.textFont(largeFont);
    p.fill(220, 245, 235);
    p.text(title, x + w / 2.0f, y + 28);
    p.popStyle();
  }

  void hudBadge(float x, float y, float w, float h, String value, String caption, color c) {
    p.pushStyle();
    p.stroke(c);
    p.strokeWeight(1.2f);
    p.fill(p.red(c), p.green(c), p.blue(c), 18);
    p.rect(x, y, w, h, 6);
    p.textAlign(PConstants.CENTER);
    p.textFont(font);
    p.fill(c);
    p.text(value, x + w / 2.0f, y + 22);
    p.fill(c);
    p.text(caption, x + w / 2.0f, y + h - 10);
    p.popStyle();
  }

  void drawMetric(float x, float y, String label, String value, color c) {
    p.textAlign(PConstants.LEFT);
    p.textFont(font);
    p.fill(170, 210, 195);
    p.text(label, x, y);
    p.fill(c);
    p.text(value, x + 118, y);
  }

  void drawTurretSilhouette(float x, float y) {
    p.pushStyle();

    color stateColor = alertColor();
    float yawOffset = p.map(animatedTurretAzimuth, minAzimuth, maxAzimuth, -48, 48);
    float pitchOffset = p.map(animatedTurretElevation, 20, 90, 20, -28);
    float phase = p.millis() * 0.012f;
    boolean firingNow = p.millis() < manualFireFlashUntilMs;
    boolean trackingNow = iState == 3;

    p.noFill();
    p.stroke(40, 255, 120, 55);
    p.strokeWeight(1);
    p.arc(x - 4, y + 64, 150, 42, p.radians(205), p.radians(335));
    p.stroke(hudBlue(), 130);
    p.line(x - 4 + yawOffset, y + 45, x - 4 + yawOffset, y + 84);

    p.noStroke();
    p.fill(38, 48, 46);
    p.rect(x - 72, y + 82, 144, 12, 4);
    p.fill(72, 84, 78);
    p.rect(x - 54, y + 62, 108, 22, 6);
    p.fill(32, 40, 38);
    p.ellipse(x, y + 58, 82, 28);

    p.pushMatrix();
    p.translate(x + yawOffset, y + 36);

    p.noStroke();
    p.fill(82, 94, 88);
    p.ellipse(0, 20, 66, 44);
    p.fill(125, 136, 126);
    p.ellipse(0, 12, 46, 38);
    p.fill(28, 34, 34);
    p.ellipse(0, 12, 22, 22);

    float barrelEndX = 102;
    float barrelEndY = pitchOffset;
    p.stroke(205, 218, 196);
    p.strokeWeight(12);
    p.line(12, 9, barrelEndX, barrelEndY);
    p.stroke(80, 92, 86);
    p.strokeWeight(16);
    p.line(15, 13, 56, 8 + pitchOffset * 0.35f);
    p.stroke(225, 235, 215);
    p.strokeWeight(7);
    p.line(48, 8 + pitchOffset * 0.35f, barrelEndX, barrelEndY);

    p.noStroke();
    p.fill(stateColor);
    p.ellipse(0, 12, 8 + sin(phase) * 2, 8 + sin(phase) * 2);

    if (trackingNow || firingNow) {
      int beamAlpha = firingNow ? 210 : 110;
      p.stroke(p.red(stateColor), p.green(stateColor), p.blue(stateColor), beamAlpha);
      p.strokeWeight(firingNow ? 5 : 3);
      p.line(barrelEndX, barrelEndY, barrelEndX + 78, barrelEndY - 18);
      p.stroke(p.red(stateColor), p.green(stateColor), p.blue(stateColor), firingNow ? 65 : 30);
      p.strokeWeight(firingNow ? 14 : 9);
      p.line(barrelEndX + 6, barrelEndY - 1, barrelEndX + 88, barrelEndY - 21);
      p.noStroke();
      p.fill(p.red(stateColor), p.green(stateColor), p.blue(stateColor), firingNow ? 210 : 120);
      p.ellipse(barrelEndX + 82, barrelEndY - 19, firingNow ? 14 : 8, firingNow ? 14 : 8);
    }

    p.popMatrix();

    p.popStyle();
  }

  void drawHeader() {
    p.textAlign(PConstants.CENTER);
    p.textFont(extraLargeFont);
    p.fill(hudBlue());
    p.text("نموذج مصغر يحاكي منظومة دفاع جوي ورادار إنذار مبكر 360°", p.width / 2.0f, 45);

    float rightHeaderCenterX = p.width - 199;
    float headerMiddleX = rightHeaderCenterX + 10;
    p.textAlign(PConstants.CENTER);
    p.textFont(font);
    p.fill(hudBlue());
    p.text("الدفعة 34 دفاع جوي /", rightHeaderCenterX, 34);

    p.textAlign(PConstants.CENTER);
    p.fill(220, 240, 230);
    p.text("إعداد", headerMiddleX, 58);

    p.textAlign(PConstants.CENTER);
    p.textFont(largeFont);
    p.fill(255, 70, 60);
    p.text("المهندس: عماد الصبري", rightHeaderCenterX, 84);
  }

  void drawMainRadar() {
    p.pushMatrix();
    p.translate(radarCenter.x, radarCenter.y);
    drawEnhancedRadarGrid();
    drawRadarPoints();
    drawTurretIndicator();
    drawSweepIndicator();
    p.popMatrix();
  }

  void drawEnhancedRadarGrid() {
    p.noStroke();
    int baseGlowAlpha = iState == 3 ? 8 : 5;
    for (int glow = 0; glow < 10; glow++) {
      color haloColor = radarOrange();
      p.fill(p.red(haloColor), p.green(haloColor), p.blue(haloColor), baseGlowAlpha);
      float d = radarRadius * 2 + glow * 12;
      p.ellipse(0, 0, d, d);
    }

    p.strokeWeight(1.4f);
    int ringCount = 9;

    for (int i = 1; i <= ringCount; i++) {
      float distance = i * 50;
      float rr = p.map(distance, 0, displayMaxRangeCm, 0, radarRadius);

      color ringColor = (i == ringCount) ? p.color(110, 190, 255) : getRangeColor(distance);
      p.stroke(p.red(ringColor), p.green(ringColor), p.blue(ringColor), i == ringCount ? 210 : 95);
      p.noFill();
      p.ellipse(0, 0, rr * 2, rr * 2);

      p.fill(i == ringCount ? p.color(170, 225, 255) : p.color(130, 175, 155));
      p.textSize(11);
      p.textAlign(PConstants.CENTER);
      p.text(int(distance) + "cm", 0, rr - 4);
      p.text(int(distance) + "cm", rr - 24, (i % 2 == 0) ? -8 : 10);
    }

    for (int angle = 0; angle < 360; angle += 10) {
      boolean axisLine = angle % 90 == 0;
      p.stroke(80, 255, 130, axisLine ? 160 : 65);
      p.strokeWeight(axisLine ? 2.2f : 0.9f);
      float rad = p.radians(angle - 90);
      p.line(0, 0, p.cos(rad) * radarRadius, p.sin(rad) * radarRadius);

      if (angle % 30 == 0) {
        p.fill(205, 230, 220);
        p.textSize(axisLine ? 16 : 13);
      p.text(angle + "°", p.cos(rad) * (radarRadius + 28), p.sin(rad) * (radarRadius + 28));
      }
    }

    for (int angle = 0; angle < 360; angle += 2) {
      float rad = p.radians(angle - 90);
      boolean majorTick = angle % 10 == 0;
      boolean midTick = angle % 5 == 0;
      float tickLen = majorTick ? 18 : (midTick ? 12 : 7);
      float outerR = radarRadius + 2;
      float innerR = outerR - tickLen;
      int alpha = majorTick ? 245 : (midTick ? 145 : 70);
      p.stroke(majorTick ? hudBlue() : hudGreen(), alpha);
      p.strokeWeight(majorTick ? 2.3f : (midTick ? 1.35f : 1.0f));
      p.line(
        p.cos(rad) * innerR,
        p.sin(rad) * innerR,
        p.cos(rad) * outerR,
        p.sin(rad) * outerR
      );
    }

    p.stroke(40, 255, 120, 230);
    p.strokeWeight(2);
    p.line(-14, 0, 14, 0);
    p.line(0, -14, 0, 14);
    p.fill(20, 255, 90);
    p.noStroke();
    p.ellipse(0, 0, 10, 10);
  }

  void drawRadarPoints() {
    synchronized (pings) {
      for (RadarPing ping : pings) {
        float a = p.radians(ping.angle - 90);
        float rr = p.map(ping.distance, 0, displayMaxRangeCm, 0, radarRadius);
        float x = p.cos(a) * rr;
        float y = p.sin(a) * rr;
        int alpha = (int)p.constrain(p.map(p.millis() - ping.timestamp, 0, pingLifetimeMs(ping.distance), 255, 0), 0, 255);

        color c = getRangeColor(ping.distance);
        p.noStroke();
        p.fill(p.red(c), p.green(c), p.blue(c), alpha * 0.45f);
        p.ellipse(x, y, 10, 10);
        p.fill(c, alpha);
        p.ellipse(x, y, 5, 5);
      }
    }

    if (isValidDistance(iDistance)) {
      float targetRad = p.radians(iAngle - 90);
      float targetR = p.map(iDistance, 0, displayMaxRangeCm, 0, radarRadius);
      float x = p.cos(targetRad) * targetR;
      float y = p.sin(targetRad) * targetR;
      color lockColor = alertColor();

      p.stroke(p.red(lockColor), p.green(lockColor), p.blue(lockColor), 190);
      p.strokeWeight(1.5f);
      p.line(0, 0, x, y);
      p.noStroke();
      p.fill(p.red(lockColor), p.green(lockColor), p.blue(lockColor), 80);
      p.ellipse(x, y, 24, 24);
      p.fill(lockColor);
      p.ellipse(x, y, 8, 8);

      p.stroke(p.red(lockColor), p.green(lockColor), p.blue(lockColor), 210);
      p.fill(5, 18, 18, 228);
      float labelX = p.constrain(x + 26, -radarRadius + 30, radarRadius - 115);
      float labelY = p.constrain(y - 34, -radarRadius + 18, radarRadius - 50);
      p.rect(labelX, labelY, 112, 42, 5);
      p.textAlign(PConstants.CENTER);
      p.textFont(font);
      p.fill(lockColor);
      p.text("TARGET LOCK", labelX + 56, labelY + 17);
      p.fill(255);
      p.text(iDistance + " cm", labelX + 56, labelY + 34);
    }
  }

  void drawTurretIndicator() {
    int turretRadarAngle = azimuthToRadarAngle(iAzimuth);
    float azimuthRad = p.radians(turretRadarAngle - 90);

    color c = alertColor();

    float sighterLen = radarRadius * 0.85f;
    p.stroke(p.red(c), p.green(c), p.blue(c), 210);
    p.strokeWeight(iState == 3 ? 7 : 4);
    p.line(0, 0, p.cos(azimuthRad) * sighterLen, p.sin(azimuthRad) * sighterLen);

    float tx = p.cos(azimuthRad) * sighterLen;
    float ty = p.sin(azimuthRad) * sighterLen;
    p.noStroke();
    p.fill(p.red(c), p.green(c), p.blue(c), 60);
    p.ellipse(tx, ty, 22, 22);
    p.fill(255);
    p.ellipse(tx, ty, 7, 7);
    p.fill(120, 215, 255);
    p.textAlign(PConstants.CENTER);
    p.textSize(12);
    p.text("ELV " + iElevation + "°", tx, ty - 16);
  }

  void drawSweepIndicator() {
    p.noStroke();
    for (int i = 0; i < 34; i++) {
      float sweepAngle = currentAngle - i * 1.35f;
      if (sweepAngle < 0) sweepAngle += 360;
      float startAngle = p.radians(sweepAngle - 90);
      float endAngle = p.radians(sweepAngle - 91.35f);
      int alpha = (int)p.map(i, 0, 34, 180, 0);
      p.fill(60, 255, 100, alpha * 0.35f);
      p.arc(0, 0, radarRadius * 2, radarRadius * 2, endAngle, startAngle);
    }

    float angleRad = p.radians(currentAngle - 90);
    p.stroke(80, 255, 120, 240);
    p.strokeWeight(3);
    p.line(0, 0, p.cos(angleRad) * radarRadius, p.sin(angleRad) * radarRadius);
  }

  void drawLeftInfoPanel() {
    float x = 38;
    float w = 322;

    hudPanel(x, 92, w, 118, "حالة الاتصال");
    p.textAlign(PConstants.LEFT);
    p.textFont(largeFont);
    p.fill(isConnected ? hudGreen() : p.color(255, 75, 65));
    p.ellipse(x + 34, 146, 13, 13);
    p.text((isConnected ? "متصل - " : "غير متصل - ") + portName, x + 58, 152);
    p.textFont(font);
    p.fill(rawSerialColor());
    p.text(rawSerialLabel(), x + 24, 174);
    p.fill(radarPacketColor());
    p.text(radarPacketLabel(), x + 162, 174);
    p.fill(170, 210, 205);
    p.text(shortRawSerialLine(), x + 24, 194);

    hudPanel(x, 224, w, 194, "مستوى التهديد");
    p.textAlign(PConstants.LEFT);
    p.textFont(largeFont);
    p.fill(alertColor());
    p.text(modeLabel(), x + 24, 272);
    p.textFont(font);
    p.fill(alertColor());
    p.text(threatLevelText, x + 24, 294, w - 48, 34);
    drawMetric(x + 24, 338, "المسافة", distanceLabel(iDistance), p.color(255));
    drawMetric(x + 24, 362, "الزاوية", iAngle + "°", p.color(255));
    drawMetric(x + 24, 386, "آخر رصد", getLastTargetTime(), p.color(255));
    drawMetric(x + 24, 410, "التاريخ", getLastTargetDate(), p.color(255));

    hudPanel(x, 430, w, 226, "النظام المسلح");
    drawTurretSilhouette(x + 148, 486);
    p.textFont(font);
    p.fill(hudBlue());
    p.textAlign(PConstants.LEFT);
    p.text("الاتجاه: " + iAzimuth + "°", x + 24, 622);
    p.textAlign(PConstants.RIGHT);
    p.text("الارتفاع: " + iElevation + "°", x + w - 24, 622);
    p.textAlign(PConstants.CENTER);
    p.textFont(largeFont);
    p.fill(alertColor());
    p.text(alarmStatusText, x + w / 2.0f, 648);

    hudPanel(x, 664, w, 152, "الوضع اليدوي");
    drawManualControlPanel(x, 664, w);
  }

  void drawRightInfoPanel() {
    float w = 322;
    float x = p.width - w - 38;

    hudPanel(x, 118, w, 132, "مؤشر الحالة");
    hudBadge(x + 28, 166, 78, 58, "SCAN", "مسح", iState >= 1 ? hudBlue() : p.color(65, 100, 115));
    hudBadge(x + 122, 166, 78, 58, "TRACK", "تتبع", iState == 3 ? p.color(255, 210, 70) : p.color(90, 95, 55));
    hudBadge(x + 216, 166, 78, 58, autoControlEnabled ? "AUTO" : "MANUAL", "تحكم", autoControlEnabled ? p.color(255, 210, 70) : p.color(255, 70, 55));

    hudPanel(x, 274, w, 234, "HUD عسكري");
    drawMiniHud(x + 22, 316, w - 44, 150);
    drawMetric(x + 28, 486, "محور الرادار", iAngle + "°", hudBlue());
    drawMetric(x + 170, 486, "البرج", iAzimuth + "°", hudGreen());

    hudPanel(x, 532, w, 210, "سجل التتبع");
    drawTrackHistory(x + 26, 582, w - 52, 118);

  }

  void drawManualControlPanel(float x, float y, float w) {
    color modeColor = manualControlColor();
    drawManualKeyHint(x + 24, y + 64, "A", manualModeLabel(), modeColor);
    drawManualKeyHint(x + 92, y + 64, "F", "إطلاق", modeColor);
    drawManualKeyHint(x + 160, y + 64, "S", "إيقاف", hudGreen());
    drawManualKeyHint(x + 238, y + 46, "←→", "يمين/يسار", modeColor);
    drawManualKeyHint(x + 238, y + 90, "↑↓", "فوق/تحت", modeColor);
  }

  color manualControlColor() {
    if (autoControlEnabled) return p.color(255, 210, 70);
    return p.color(255, 70, 55);
  }

  String manualModeLabel() {
    if (autoControlEnabled) return "تلقائي";
    return "يدوي";
  }

  void drawManualKeyHint(float x, float y, String keyName, String label, color c) {
    p.pushStyle();
    p.stroke(c);
    p.strokeWeight(1.05f);
    p.fill(p.red(c), p.green(c), p.blue(c), 24);
    p.rect(x, y, 44, 24, 5);
    p.textAlign(PConstants.CENTER);
    p.textFont(font);
    p.fill(c);
    p.text(keyName, x + 22, y + 18);
    p.fill(220, 245, 235);
    p.text(label, x + 22, y + 41);
    p.popStyle();
  }

  void drawKeyHint(float x, float y, String keyName, String label, color c) {
    p.pushStyle();
    p.stroke(c);
    p.strokeWeight(1.1f);
    p.fill(p.red(c), p.green(c), p.blue(c), 22);
    p.rect(x, y, 42, 28, 5);
    p.textAlign(PConstants.CENTER);
    p.textFont(largeFont);
    p.fill(c);
    p.text(keyName, x + 21, y + 21);
    p.textFont(font);
    p.text(label, x + 21, y + 52);
    p.popStyle();
  }

  void drawMiniHud(float x, float y, float w, float h) {
    p.pushStyle();
    p.stroke(40, 105, 85);
    p.fill(2, 18, 14, 190);
    p.rect(x, y, w, h, 4);

    float cx = x + w / 2.0f;
    float cy = y + h / 2.0f;
    float r = min(w, h) * 0.42f;

    p.noFill();
    p.stroke(30, 180, 80, 90);
    for (int i = 1; i <= 3; i++) {
      p.ellipse(cx, cy, r * i / 1.5f, r * i / 1.5f);
    }

    p.stroke(60, 255, 100, 170);
    p.line(cx - r, cy, cx + r, cy);
    p.line(cx, cy - r, cx, cy + r);

    float turretRad = p.radians(azimuthToRadarAngle(iAzimuth) - 90);
    p.stroke(255, 215, 75, 210);
    p.line(cx, cy, cx + p.cos(turretRad) * r, cy + p.sin(turretRad) * r);

    if (isValidDistance(iDistance)) {
      float targetRad = p.radians(iAngle - 90);
      float targetR = p.map(iDistance, 0, displayMaxRangeCm, 0, r);
      p.noStroke();
      p.fill(255, 80, 60);
      p.ellipse(cx + p.cos(targetRad) * targetR, cy + p.sin(targetRad) * targetR, 9, 9);
    }

    p.popStyle();
  }

  void drawTrackHistory(float x, float y, float w, float h) {
    p.pushStyle();
    p.stroke(35, 105, 85);
    p.fill(2, 18, 14, 190);
    p.rect(x, y, w, h, 4);

    float plotX = x + 36;
    float plotY = y + 10;
    float plotW = w - 48;
    float plotH = h - 34;

    p.stroke(25, 130, 90, 95);
    p.fill(150, 190, 175);
    p.textFont(font);
    p.textAlign(PConstants.RIGHT);
    for (int cm = 0; cm <= 400; cm += 100) {
      float gy = p.map(cm, 0, 400, plotY + plotH, plotY);
      p.line(plotX, gy, plotX + plotW, gy);
      p.text(cm, plotX - 6, gy + 4);
    }

    p.textAlign(PConstants.CENTER);
    for (int angle = 0; angle <= 360; angle += 90) {
      float gx = p.map(angle, 0, 360, plotX, plotX + plotW);
      p.line(gx, plotY, gx, plotY + plotH);
      p.text(angle, gx, plotY + plotH + 18);
    }

    synchronized (pings) {
      int start = max(0, pings.size() - 26);
      float lastX = 0;
      float lastY = 0;
      boolean hasLast = false;
      for (int i = start; i < pings.size(); i++) {
        RadarPing ping = pings.get(i);
        float px = p.map(wrapAngle360((int)ping.angle), 0, 360, plotX, plotX + plotW);
        float py = p.map(p.constrain(ping.distance, 0, 400), 0, 400, plotY + plotH, plotY);
        color c = getRangeColor(ping.distance);
        if (hasLast) {
          p.stroke(255, 205, 70, 160);
          p.line(lastX, lastY, px, py);
        }
        p.noStroke();
        p.fill(c);
        p.ellipse(px, py, 6, 6);
        lastX = px;
        lastY = py;
        hasLast = true;
      }
    }
    p.popStyle();
  }

  String distanceLabel(int value) {
    if (isValidDistance(value)) return value + " سم";
    if (value == 0) return "منطقة عمياء <= 20 سم";
    if (value < 0) return "لا يوجد صدى";
    if (value > displayMaxRangeCm) return "خارج المدى";
    return "---";
  }

  void drawStatusBar() {
    float y = p.height - 78;
    float h = 54;
    float margin = 38;
    float gap = 12;
    float boxW = (p.width - margin * 2.0f - gap * 6.0f) / 7.0f;
    String[] labels = {"STATUS", "TARGETS", "ENGAGED", "MODE", "ANGLE", "RANGE", "FIRE"};
    String[] values = {
      connectionArabicLabel(),
      totalDetectedTargets + " مكتشف",
      engagedTargetCount + " مستهدف",
      modeArabicLabel() + " / " + autoFireArabicLabel(),
      "رادار " + iAngle + "° / برج " + iAzimuth + "°",
      distanceLabel(iDistance),
      fireArabicLabel() + " - " + controlStatusText
    };
    color[] colors = {
      isConnected ? hudGreen() : p.color(255, 70, 55),
      hudBlue(),
      p.color(255, 210, 70),
      alertColor(),
      p.color(230, 240, 220),
      p.color(230, 240, 220),
      iState == 3 ? p.color(255, 70, 55) : hudGreen()
    };

    float x = margin;
    p.textFont(font);
    p.textSize(12);
    for (int i = 0; i < labels.length; i++) {
      float w = boxW;
      p.stroke(hudBlue(), 130);
      p.strokeWeight(1.2f);
      p.fill(5, 18, 18, 225);
      p.rect(x, y, w, h, 6);
      p.textAlign(PConstants.LEFT);
      p.fill(hudBlue());
      p.text(labels[i] + ":", x + 18, y + 34);
      p.fill(colors[i]);
      p.text(values[i], x + 92, y + 34);
      x += w + gap;
    }
  }

  void handleVisualAlarm() {
    if (iState == 3 && p.frameCount % 20 < 10) {
      p.pushMatrix();
      p.translate(radarCenter.x, radarCenter.y);
      p.noStroke();
      p.fill(255, 95, 10, 63);
      p.ellipse(0, 0, radarRadius * 2, radarRadius * 2);
      p.popMatrix();
    }
  }

  void updateStateTexts() {
    if (iState == 0) {
      threatLevelText = "لا يوجد هدف صالح أو الهدف داخل المنطقة العمياء";
      alarmStatusText = "خمول / منطقة عمياء";
      targetDetected = false;
    } else if (iState == 1) {
      threatLevelText = "تم رصد هدف بعيد (من 301 إلى 500 سم)";
      alarmStatusText = "إنذار بعيد";
      if (!targetDetected && isValidDistance(iDistance)) recordLastTargetTime();
      targetDetected = true;
    } else if (iState == 2) {
      threatLevelText = "تم رصد هدف متوسط المدى (من 201 إلى 300 سم)";
      alarmStatusText = "إنذار متوسط";
      if (!targetDetected && isValidDistance(iDistance)) recordLastTargetTime();
      targetDetected = true;
    } else if (iState == 3) {
      threatLevelText = "تم رصد هدف خطر مع تتبع وتصويب (من 41 إلى 200 سم)";
      alarmStatusText = "خطر مع تتبع";
      if (!targetDetected && isValidDistance(iDistance)) recordLastTargetTime();
      targetDetected = true;
    }
  }

  color getRangeColor(float distance) {
    if (distance <= 200) return p.color(255, 70, 50);
    if (distance <= 300) return p.color(255, 235, 80);
    return p.color(80, 255, 130);
  }

  boolean isValidDistance(int value) {
    return value > 40 && value <= displayMaxRangeCm;
  }

  int wrapAngle360(int value) {
    while (value < 0) value += 360;
    while (value >= 360) value -= 360;
    return value;
  }

  float angleDelta(float fromAngle, float toAngle) {
    float diff = wrapAngle360((int)toAngle) - wrapAngle360((int)fromAngle);
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
  }

  int turretForwardRadarAngle() {
    return wrapAngle360(radarForwardAngle + turretForwardRadarOffsetDeg);
  }

  int azimuthToRadarAngle(int azimuth) {
    azimuth = (int)p.constrain(azimuth, minAzimuth, maxAzimuth);
    int relativeAngle;
    int effectiveLeftSpan = radarAimLeftSpan;
    int effectiveRightSpan = radarAimRightSpan;

    if (azimuthDirectionInverted) {
      effectiveLeftSpan = radarAimRightSpan;
      effectiveRightSpan = radarAimLeftSpan;
    }

    if (homeAzimuth <= minAzimuth && azimuth <= homeAzimuth) {
      relativeAngle = 0;
    } else if (azimuth <= homeAzimuth) {
      relativeAngle = (int)p.map(azimuth, minAzimuth, homeAzimuth, -effectiveLeftSpan, 0);
    } else {
      relativeAngle = (int)p.map(azimuth, homeAzimuth, maxAzimuth, 0, effectiveRightSpan);
    }

    if (azimuthDirectionInverted) {
      relativeAngle = -relativeAngle;
    }

    return wrapAngle360(turretForwardRadarAngle() + relativeAngle);
  }

  String currentClockText() {
    int hour24 = p.hour();
    int hour12 = hour24 % 12;
    if (hour12 == 0) hour12 = 12;
    String period = hour24 >= 12 ? "PM" : "AM";
    return nf(hour12, 2) + ":" + nf(p.minute(), 2) + ":" + nf(p.second(), 2) + " " + period;
  }

  String currentDateText() {
    return nf(p.day(), 2) + "/" + nf(p.month(), 2) + "/" + p.year();
  }

  void recordLastTargetTime() {
    lastTargetClockText = currentClockText();
    lastTargetDateText = currentDateText();
  }

  String getLastTargetTime() {
    return lastTargetClockText;
  }

  String getLastTargetDate() {
    return lastTargetDateText;
  }
}
