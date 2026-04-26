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

  final int displayMaxRangeCm = 400;
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
  boolean targetDetected = false;
  int totalDetectedTargets = 0;

  EnhancedRadarSystem(PApplet p) {
    this.p = p;
    radarCenter = new PVector(p.width * 0.48f, p.height * 0.47f);
    radarRadius = min(p.width * 0.22f, p.height * 0.32f);
  }

  void initialize() {
    font = p.createFont("Tahoma", 14);
    largeFont = p.createFont("Tahoma", 18);
    extraLargeFont = p.createFont("Tahoma Bold", 26);

    println("Available serial ports:");
    printAvailablePorts();
    setupSerial();
    p.background(10, 15, 20);
  }

  void update() {
    serviceSerialInput();
    p.background(10, 15, 20);
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
      if (!targetDetected && isValidDistance(distance)) {
        lastTargetClockText = currentClockText();
      }
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

  void drawHeader() {
    color codeBlue = p.color(74, 144, 226);

    p.textAlign(PConstants.CENTER);
    p.textFont(extraLargeFont);
    p.fill(codeBlue);
    p.text("نظام الرادار الذكي", p.width/2, 45);

    p.textAlign(PConstants.RIGHT);
    p.textFont(largeFont);
    p.fill(codeBlue);
    p.text("الدفعة 34 دفاع جوي /", p.width - 60, 35);

    p.fill(255);
    p.text("إعداد", p.width - 60, 62);

    p.fill(255, 50, 50);
    p.text("المهندس : عماد الصبري", p.width - 60, 89);

    p.textAlign(PConstants.CENTER);
    p.textFont(font);
    p.fill(100, 200, 100);
    p.text("دوران شامل - 360°", p.width/2, 82);
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
    p.strokeWeight(2);
    float ringStep = radarRadius / 8.0f;

    for (int i = 1; i <= 8; i++) {
      float distance = i * 50;
      float rr = p.map(distance, 0, displayMaxRangeCm, 0, radarRadius);

      color ringColor = (i == 8) ? p.color(110, 190, 255) : getRangeColor(distance);
      p.stroke(p.red(ringColor), p.green(ringColor), p.blue(ringColor), 170);
      p.noFill();
      p.ellipse(0, 0, rr * 2, rr * 2);

      p.fill(200, 220, 255);
      p.textSize(12);
      p.textAlign(PConstants.CENTER);
      String distLabel = int(distance) + " سم";

      float labelRadius = rr - ringStep;
      if (labelRadius < ringStep * 0.6f) labelRadius = ringStep * 0.6f;

      if (distance == 50) {
        p.fill(255, 220, 120);
        p.textSize(14);
        p.text(distLabel, 0, -14);
      } else {
        p.text(distLabel, 0, -labelRadius - 8);
        p.text(distLabel, 0, labelRadius + 15);
        p.textAlign(PConstants.LEFT);
        p.text(distLabel, labelRadius + 5, 0);
        p.textAlign(PConstants.RIGHT);
        p.text(distLabel, -labelRadius - 5, 0);
        p.textAlign(PConstants.CENTER);
      }
    }

    for (int angle = 0; angle < 360; angle += 30) {
      p.stroke(130, 190, 255, (angle % 90 == 0) ? 240 : 150);
      p.strokeWeight((angle % 90 == 0) ? 3 : 1);
      float rad = p.radians(angle - 90);
      p.line(0, 0, p.cos(rad) * radarRadius, p.sin(rad) * radarRadius);

      p.fill(255);
      p.textSize(16);
      p.text(angle + "°", p.cos(rad) * (radarRadius + 30), p.sin(rad) * (radarRadius + 30));
    }

    p.fill(255, 220, 0);
    p.noStroke();
    p.ellipse(0, 0, 12, 12);
  }

  void drawRadarPoints() {
    synchronized (pings) {
      for (RadarPing ping : pings) {
        float a = p.radians(ping.angle - 90);
        float rr = p.map(ping.distance, 0, displayMaxRangeCm, 0, radarRadius);
        float x = p.cos(a) * rr;
        float y = p.sin(a) * rr;
        int alpha = (int)p.map(p.millis() - ping.timestamp, 0, pingLifetimeMs(ping.distance), 255, 0);

        color c = getRangeColor(ping.distance);
        p.noStroke();
        p.fill(p.red(c), p.green(c), p.blue(c), alpha * 0.65f);
        p.ellipse(x, y, 20, 20);
        p.fill(c, alpha);
        p.ellipse(x, y, 8, 8);
      }
    }
  }

  void drawTurretIndicator() {
    int turretRadarAngle = azimuthToRadarAngle(iAzimuth);
    float azimuthRad = p.radians(turretRadarAngle - 90);

    if (isValidDistance(iDistance)) {
      float targetRad = p.radians(iAngle - 90);
      float targetR = p.map(iDistance, 0, displayMaxRangeCm, 0, radarRadius);
      p.stroke(255, 0, 0, 100);
      p.strokeWeight(1);
      p.line(0, 0, p.cos(targetRad) * targetR, p.sin(targetRad) * targetR);
    }

    if (iState == 3) p.stroke(255, 70, 50, 245);
    else if (iState == 2) p.stroke(255, 235, 80, 220);
    else if (iState == 1) p.stroke(80, 255, 130, 220);
    else p.stroke(100, 190, 255, 170);

    float sighterLen = radarRadius * 0.85f;
    p.strokeWeight(iState == 3 ? 8 : 4);
    p.line(0, 0, p.cos(azimuthRad) * sighterLen, p.sin(azimuthRad) * sighterLen);

    float tx = p.cos(azimuthRad) * sighterLen;
    float ty = p.sin(azimuthRad) * sighterLen;
    p.noStroke();
    p.fill(255);
    p.ellipse(tx, ty, 10, 10);
    p.fill(0, 180, 255);
    p.textAlign(PConstants.CENTER);
    p.textSize(12);
    p.text("ELV: " + iElevation + "°", tx, ty - 15);
  }

  void drawSweepIndicator() {
    p.noStroke();
    for (int i = 0; i < 25; i++) {
      float sweepAngle = currentAngle - i;
      if (sweepAngle < 0) sweepAngle += 360;
      float startAngle = p.radians(sweepAngle - 90);
      float endAngle = p.radians(sweepAngle - 91);
      int alpha = (int)p.map(i, 0, 25, 255, 0);
      p.fill(80, 255, 120, alpha * 0.42f);
      p.arc(0, 0, radarRadius * 2, radarRadius * 2, endAngle, startAngle);
    }

    float angleRad = p.radians(currentAngle - 90);
    p.stroke(90, 255, 130, 240);
    p.strokeWeight(4);
    p.line(0, 0, p.cos(angleRad) * radarRadius, p.sin(angleRad) * radarRadius);
  }

  void drawLeftInfoPanel() {
    p.pushMatrix();
    p.translate(42, 110);
    p.stroke(80);
    p.fill(20, 25, 30, 220);
    p.rect(0, 0, 320, 74, 8);
    p.fill(180, 220, 255);
    p.textFont(font);
    p.textAlign(PConstants.LEFT);
    p.text("حالة الاتصال:", 15, 25);
    p.fill(isConnected ? p.color(100, 255, 100) : p.color(255, 100, 100));
    p.text((isConnected ? "متصل " : "غير متصل ") + connectionStatus, 15, 52);
    p.popMatrix();
  }

  void drawRightInfoPanel() {
    float x = p.width - 390;
    float y = 110;
    p.pushMatrix();
    p.translate(x, y);
    p.stroke(100);
    p.fill(20, 25, 30, 220);
    p.rect(0, 0, 320, 410, 8);

    p.textAlign(PConstants.LEFT);
    p.textFont(largeFont);
    p.fill(180, 220, 255);
    p.text("معلومات البرج والأهداف", 20, 30);

    p.textFont(font);
    int ly = 65;
    p.fill(200, 255, 200);
    p.text("زاوية عين الرادار: " + iAngle + "°", 20, ly); ly += 30;
    p.text("مسافة الهدف: " + distanceLabel(iDistance), 20, ly); ly += 30;

    color statusColor = p.color(100, 180, 255);
    if (iState == 3) statusColor = p.color(255, 50, 50);
    else if (iState == 2) statusColor = p.color(255, 255, 0);
    else if (iState == 1) statusColor = p.color(50, 255, 50);

    p.fill(statusColor);
    p.text("المنظومة: " + alarmStatusText, 20, ly); ly += 30;
    p.text("التنشط: " + threatLevelText, 20, ly); ly += 35;

    p.fill(200, 220, 255);
    p.text("اتجاه برج الإطلاق: " + iAzimuth + "°", 20, ly); ly += 30;
    p.text("ارتفاع الإطلاق: " + iElevation + "°", 20, ly); ly += 30;

    float dangerLevel = isValidDistance(iDistance) ? p.map(iDistance, 0, displayMaxRangeCm, 250, 0) : 0;
    dangerLevel = p.constrain(dangerLevel, 0, 250);
    float mix = isValidDistance(iDistance) ? p.map(iDistance, 0, displayMaxRangeCm, 0, 1) : 1;
    color barColor = p.lerpColor(p.color(255, 0, 0), p.color(0, 255, 0), mix);
    p.noStroke();
    p.fill(barColor);
    p.rect(20, ly, dangerLevel, 10, 5); ly += 25;

    p.fill(255);
    p.text("مستوى الخطر", 20, ly); ly += 30;

    p.fill(200, 220, 255);
    p.text("وقت آخر رصد: " + getLastTargetTime(), 20, ly); ly += 30;

    p.fill(100, 255, 100);
    p.text("كمية الأهداف بالسجل: " + totalDetectedTargets, 20, ly);

    if (iAngle > 180 && iState > 0) {
      ly += 30;
      p.fill(255, 100, 50);
      p.text("الهدف خارج النطاق (خلف البرج)", 10, ly);
    }

    p.popMatrix();
  }

  String distanceLabel(int value) {
    if (isValidDistance(value)) return value + " سم";
    if (value == 0) return "منطقة عمياء <= 20 سم";
    if (value < 0) return "لا يوجد صدى";
    if (value > displayMaxRangeCm) return "خارج المدى";
    return "---";
  }

  void drawStatusBar() {
    p.fill(0, 0, 0, 165);
    p.noStroke();
    p.rect(0, p.height - 86, p.width, 86);

    p.fill(0, 255, 0);
    p.textSize(16);
    p.textAlign(PConstants.LEFT);
    p.text("محور العدسة: " + iAngle + "°", 20, p.height - 50);
    p.text("المنفذ: " + portName, 20, p.height - 22);

    p.textAlign(PConstants.CENTER);
    p.text("المسافة: " + distanceLabel(iDistance), p.width/2.0f, p.height - 50);
    p.text("توجيه المدفع: " + iAzimuth + "° | الارتفاع: " + iElevation + "°", p.width/2.0f, p.height - 22);

    p.textAlign(PConstants.RIGHT);
    if (iState == 3) {
      p.fill(255, 50, 50);
      p.text("التهديد: " + threatLevelText, p.width - 20, p.height - 50);
    } else {
      if (iState == 2) p.fill(255, 255, 0);
      else if (iState == 1) p.fill(0, 255, 0);
      else p.fill(100, 180, 255);
      p.text("الاستعداد: " + threatLevelText, p.width - 20, p.height - 50);
    }
    p.fill(180, 220, 255);
    p.text(alarmStatusText, p.width - 20, p.height - 22);
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
      threatLevelText = "تم رصد هدف بعيد (من 251 إلى 400 سم)";
      alarmStatusText = "إنذار بعيد";
      if (!targetDetected && isValidDistance(iDistance)) lastTargetClockText = currentClockText();
      targetDetected = true;
    } else if (iState == 2) {
      threatLevelText = "تم رصد هدف متوسط المدى (من 201 إلى 250 سم)";
      alarmStatusText = "إنذار متوسط";
      if (!targetDetected && isValidDistance(iDistance)) lastTargetClockText = currentClockText();
      targetDetected = true;
    } else if (iState == 3) {
      threatLevelText = "تم رصد هدف خطر مع تتبع وتصويب (من 21 إلى 200 سم)";
      alarmStatusText = "خطر مع تتبع";
      if (!targetDetected && isValidDistance(iDistance)) lastTargetClockText = currentClockText();
      targetDetected = true;
    }
  }

  color getRangeColor(float distance) {
    if (distance <= 200) return p.color(255, 70, 50);
    if (distance <= 250) return p.color(255, 235, 80);
    return p.color(80, 255, 130);
  }

  boolean isValidDistance(int value) {
    return value > 20 && value <= displayMaxRangeCm;
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
    return nf(p.hour(), 2) + ":" + nf(p.minute(), 2) + ":" + nf(p.second(), 2);
  }

  String getLastTargetTime() {
    return lastTargetClockText;
  }
}
