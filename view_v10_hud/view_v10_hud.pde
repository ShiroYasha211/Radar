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
  if (radar != null) {
    radar.queueSerialEvent(myPort);
  }
}

void keyPressed() {
  if (radar != null) {
    radar.handleKeyPress();
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
  final int maxSerialLinesPerFrame = 8;
  final int maxQueuedSerialLines = 80;

  ArrayList<RadarPing> pings = new ArrayList<RadarPing>();
  ArrayList<String> pendingSerialLines = new ArrayList<String>();

  int iAngle = 0;
  int iDistance = -1;
  int iState = 0;
  int iAzimuth = homeAzimuth;
  int iElevation = 45;

  float currentAngle = 0;
  float currentDistance = 0;
  float lastPlottedAngle = -1000;
  float lastPlottedDistance = -1000;
  long lastPlottedTime = 0;

  String alarmStatusText = "آمن";
  String threatLevelText = "دورية بحث روتينية";
  String lastTargetClockText = "--:--:--";
  String lastTargetDateText = "--/--/----";
  boolean targetDetected = false;
  int totalDetectedTargets = 0;

  EnhancedRadarSystem(PApplet p) {
    this.p = p;
    radarCenter = new PVector(p.width * 0.50f, p.height * 0.51f);
    radarRadius = min(p.width * 0.245f, p.height * 0.345f);
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
    serviceSerialInput();
    drawHudBackdrop();
    cleanupOldPings();
    drawHeader();
    drawMainRadar();
    drawLeftInfoPanel();
    drawRightInfoPanel();
    drawStatusBar();
    handleVisualAlarm();
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
      myPort.bufferUntil('\n');
      isConnected = true;
      lastSerialDataMs = 0;
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

  void queueSerialEvent(Serial port) {
    if (port == null || port != myPort) return;

    try {
      String data = port.readStringUntil('\n');
      if (data == null) return;
      data = data.trim();
      if (data.length() == 0) return;

      synchronized (pendingSerialLines) {
        pendingSerialLines.add(data);
        while (pendingSerialLines.size() > maxQueuedSerialLines) {
          pendingSerialLines.remove(0);
        }
      }
    } catch (Exception e) {
      isConnected = false;
      connectionStatus = "انقطع الاتصال: " + portName;
      closeSerial();
      println("Serial event error: " + e.getMessage());
    }
  }

  void handleKeyPress() {
    if (key == 'r' || key == 'R') {
      setupSerial();
    } else if (key == 'p' || key == 'P') {
      printAvailablePorts();
    } else if (key >= '0' && key <= '9') {
      preferredPortHint = "COM" + key;
      setupSerial();
      println("Preferred port changed to: " + preferredPortHint);
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

  String connectionArabicLabel() {
    if (isConnected) return "نشط";
    return "غير متصل";
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
    p.noStroke();
    p.fill(85, 95, 88);
    p.rect(x - 64, y + 60, 128, 12, 4);
    p.fill(55, 65, 62);
    p.rect(x - 46, y + 40, 92, 22, 5);
    p.fill(110, 120, 112);
    p.ellipse(x - 20, y + 22, 58, 58);
    p.fill(32, 38, 38);
    p.ellipse(x - 20, y + 22, 31, 31);
    p.stroke(255, 225, 80, 230);
    p.strokeWeight(5);
    p.line(x + 2, y + 7, x + 116, y - 45);
    p.stroke(255, 230, 110, 90);
    p.strokeWeight(12);
    p.line(x + 48, y - 14, x + 132, y - 52);
    p.popStyle();
  }

  void drawHeader() {
    p.textAlign(PConstants.CENTER);
    p.textFont(extraLargeFont);
    p.fill(hudBlue());
    p.text("نموذج مصغر يحاكي منظومة دفاع جوي ورادار إنذار مبكر", p.width / 2.0f, 45);

    p.textFont(font);
    p.fill(110, 190, 255);
    p.text("360°", p.width / 2.0f, 78);

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
    for (int glow = 0; glow < 10; glow++) {
      p.fill(20, 255, 90, 2);
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
    }

    for (int angle = 0; angle < 360; angle += 30) {
      p.stroke(80, 255, 130, (angle % 90 == 0) ? 160 : 65);
      p.strokeWeight((angle % 90 == 0) ? 2.2f : 1);
      float rad = p.radians(angle - 90);
      p.line(0, 0, p.cos(rad) * radarRadius, p.sin(rad) * radarRadius);

      p.fill(205, 230, 220);
      p.textSize(angle % 90 == 0 ? 16 : 13);
      p.text(angle + "°", p.cos(rad) * (radarRadius + 28), p.sin(rad) * (radarRadius + 28));
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

    hudPanel(x, 118, w, 98, "حالة الاتصال");
    p.textAlign(PConstants.LEFT);
    p.textFont(largeFont);
    p.fill(isConnected ? hudGreen() : p.color(255, 75, 65));
    p.ellipse(x + 34, 176, 13, 13);
    p.text((isConnected ? "متصل - " : "غير متصل - ") + portName, x + 58, 182);

    hudPanel(x, 238, w, 220, "مستوى التهديد");
    p.textAlign(PConstants.LEFT);
    p.textFont(largeFont);
    p.fill(alertColor());
    p.text(modeLabel(), x + 24, 292);
    p.textFont(font);
    p.fill(alertColor());
    p.text(threatLevelText, x + 24, 316, w - 48, 42);
    drawMetric(x + 24, 362, "المسافة", distanceLabel(iDistance), p.color(255));
    drawMetric(x + 24, 390, "الزاوية", iAngle + "°", p.color(255));
    drawMetric(x + 24, 418, "آخر رصد", getLastTargetTime(), p.color(255));
    drawMetric(x + 24, 446, "التاريخ", getLastTargetDate(), p.color(255));

    hudPanel(x, 472, w, 270, "النظام المسلح");
    drawTurretSilhouette(x + 148, 545);
    drawMetric(x + 185, 622, "الاتجاه", iAzimuth + "°", hudGreen());
    drawMetric(x + 185, 650, "الارتفاع", iElevation + "°", p.color(255, 215, 70));
    p.textAlign(PConstants.CENTER);
    p.textFont(largeFont);
    p.fill(alertColor());
    p.text(alarmStatusText, x + w / 2.0f, 710);
  }

  void drawRightInfoPanel() {
    float w = 322;
    float x = p.width - w - 38;

    hudPanel(x, 118, w, 132, "مؤشر الحالة");
    hudBadge(x + 28, 166, 78, 58, "SCAN", "مسح", iState >= 1 ? hudBlue() : p.color(65, 100, 115));
    hudBadge(x + 122, 166, 78, 58, "TRACK", "تتبع", iState == 3 ? p.color(255, 210, 70) : p.color(90, 95, 55));
    hudBadge(x + 216, 166, 78, 58, "FIRE", "إطلاق", iState == 3 ? p.color(255, 70, 55) : p.color(90, 35, 35));

    hudPanel(x, 274, w, 234, "HUD عسكري");
    drawMiniHud(x + 22, 316, w - 44, 150);
    drawMetric(x + 28, 486, "محور الرادار", iAngle + "°", hudBlue());
    drawMetric(x + 170, 486, "البرج", iAzimuth + "°", hudGreen());

    hudPanel(x, 532, w, 210, "سجل التتبع");
    drawTrackHistory(x + 26, 582, w - 52, 118);
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
    float boxW = (p.width - margin * 2.0f - gap * 5.0f) / 6.0f;
    String[] labels = {"STATUS", "TARGETS", "MODE", "ANGLE", "DISTANCE", "SYSTEM"};
    String[] values = {
      connectionArabicLabel(),
      totalDetectedTargets + " هدف",
      modeArabicLabel(),
      "رادار " + iAngle + "° / برج " + iAzimuth + "°",
      distanceLabel(iDistance),
      fireArabicLabel()
    };
    color[] colors = {
      isConnected ? hudGreen() : p.color(255, 70, 55),
      hudBlue(),
      alertColor(),
      p.color(230, 240, 220),
      p.color(230, 240, 220),
      iState == 3 ? p.color(255, 70, 55) : hudGreen()
    };

    float x = margin;
    p.textFont(font);
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
      p.fill(255, 0, 0, 40);
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
