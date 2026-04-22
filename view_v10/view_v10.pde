/*
Smart turret radar display.
Visual layout intentionally preserved while serial handling is improved.
*/

import processing.serial.*;

EnhancedRadarSystem radar;

void setup() {
    size(1400, 900);
    smooth();
    frameRate(45);

    radar = new EnhancedRadarSystem(this);
    radar.initialize();

    println("تم تشغيل نظام الرادار");
}

void draw() {
    if (radar != null) {
        radar.update();
    }
}

void serialEvent(Serial myPort) {
    if (radar != null && myPort != null) {
        radar.handleSerialEvent(myPort);
    }
}

void keyPressed() {
    if (radar != null) radar.handleKeyPress();
}

class RadarPing {
    float angle;
    float distance;
    long timestamp;

    RadarPing(float a, float d) {
        angle = a;
        distance = d;
        timestamp = millis();
    }
}

class EnhancedRadarSystem {
    PApplet p;
    Serial myPort;
    String portName = "";
    String preferredPortHint = "COM";
    int baudRate = 115200;
    final int minAzimuth = 0;
    final int maxAzimuth = 135;
    final int homeAzimuth = 60;
    final boolean azimuthDirectionInverted = true;
    final int radarForwardAngle = 0;
    final int turretForwardRadarOffsetDeg = 0;
    final int radarAimLeftSpan = homeAzimuth - minAzimuth;
    final int radarAimRightSpan = maxAzimuth - homeAzimuth;

    ArrayList<RadarPing> pings = new ArrayList<RadarPing>();
    float radarRadius;
    PVector radarCenter;

    float currentAngle = 0;
    float currentDistance = 0;
    float lastPlottedAngle = -1000;
    float lastPlottedDistance = -1000;
    long lastPlottedTime = 0;

    int iAngle, iDistance, iState, iAzimuth, iElevation;

    String threatLevelText = "دورية بحث روتينية (SCAN)";
    String alarmStatusText = "آمن";
    String lastTargetClockText = "--:--:--";

    boolean targetDetected = false;
    int totalDetectedTargets = 0;

    PFont font;
    PFont largeFont;
    PFont extraLargeFont;
    boolean isConnected = false;
    String connectionStatus = "غير متصل";

    EnhancedRadarSystem(PApplet parent) {
        this.p = parent;
        radarRadius = p.height * 0.38f;
        radarCenter = new PVector(p.width * 0.5f, p.height * 0.45f);
    }

    void initialize() {
        font = p.createFont("Arial", 14);
        largeFont = p.createFont("Arial", 18);
        extraLargeFont = p.createFont("Arial Bold", 26);

        println("Available serial ports:");
        printAvailablePorts();
        setupSerial();
        p.background(10, 15, 20);
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

    String pickBestPort() {
        String[] ports = Serial.list();
        if (ports == null || ports.length == 0) return "";

        if (preferredPortHint != null && preferredPortHint.length() > 0) {
            String hint = preferredPortHint.toLowerCase();
            for (int i = 0; i < ports.length; i++) {
                if (ports[i].toLowerCase().indexOf(hint) >= 0) {
                    return ports[i];
                }
            }
        }

        int bestIndex = -1;
        for (int i = 0; i < ports.length; i++) {
            String candidate = ports[i].toLowerCase();
            if (candidate.indexOf("usb") >= 0 || candidate.indexOf("acm") >= 0 || candidate.indexOf("arduino") >= 0) {
                bestIndex = i;
                break;
            }
        }
        if (bestIndex >= 0) return ports[bestIndex];
        return ports[0];
    }

    void closeSerial() {
        if (myPort != null) {
            try {
                myPort.stop();
            } catch (Exception e) {
            }
            myPort = null;
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
            myPort.bufferUntil('\n');
            isConnected = true;
            connectionStatus = "متصل بـ " + portName;
            println("Connected to serial port: " + portName);
        } catch (Exception e) {
            isConnected = false;
            connectionStatus = "خطأ اتصال في " + portName;
            println("Failed to connect to serial port: " + portName);
        }
    }

    void update() {
        p.fill(10, 15, 20, 20);
        p.rect(0, 0, p.width, p.height);

        cleanupOldPings();

        drawHeader();
        drawMainRadar();
        drawInfoPanels();
        drawStatusBar();

        handleVisualAlarm();
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
        p.text("الدفعة 34 دفاع جوي /", p.width - 30, 35);

        p.fill(255);
        p.text("إعداد", p.width - 30, 62);

        p.fill(255, 50, 50);
        p.text("المهندس : عماد الصبري", p.width - 30, 89);

        p.textAlign(PConstants.CENTER);
        p.textFont(font);
        p.fill(100, 200, 100);
        p.text("دوران شامل - 360°", p.width/2, 82);
    }

    void cleanupOldPings() {
        long currentTime = p.millis();
        int activeTargets = 0;

        synchronized(pings) {
            while (pings.size() > 140) pings.remove(0);

            for (int i = pings.size() - 1; i >= 0; i--) {
                if (currentTime - pings.get(i).timestamp > 1200) {
                    pings.remove(i);
                } else if (isValidDistance((int)pings.get(i).distance)) {
                    activeTargets++;
                }
            }
        }

        totalDetectedTargets = activeTargets;
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
            float rr = p.map(distance, 0, 400, 0, radarRadius);

            color ringColor = (i == 8) ? p.color(74, 144, 226) : getRangeColor(distance);
            p.stroke(p.red(ringColor), p.green(ringColor), p.blue(ringColor), 100);
            p.noFill();
            p.ellipse(0, 0, rr * 2, rr * 2);

            p.fill(200, 220, 255);
            p.textSize(12);
            p.textAlign(PConstants.CENTER);
            String distLabel = int(distance) + " سم";

            float labelRadius = rr - ringStep;
            if (distance == 50) {
                labelRadius = p.map(200, 0, 400, 0, radarRadius);
            }
            if (labelRadius < ringStep * 0.6f) {
                labelRadius = ringStep * 0.6f;
            }

            if (distance == 50) {
                p.fill(255, 220, 120);
                p.textSize(14);
                p.textAlign(PConstants.CENTER);
                p.text(distLabel, 0, -labelRadius + 18);
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
            p.stroke(100, 150, 255, (angle % 90 == 0) ? 200 : 100);
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

    color getRangeColor(float distance) {
        if (distance <= 150) return p.color(255, 0, 0);
        else if (distance <= 250) return p.color(255, 255, 0);
        else return p.color(0, 255, 0);
    }

    boolean isValidDistance(int value) {
        return value > 20 && value <= 400;
    }

    String distanceLabelLegacy(int value) {
        if (isValidDistance(value)) return value + " سم";
        if (value < 0) return "لا صدى";
        if (value > 400) return "خارج المدى";
        return "---";
    }

    int wrapAngle360(int value) {
        while (value < 0) value += 360;
        while (value >= 360) value -= 360;
        return value;
    }

    int angleDiffDegrees(int fromAngle, int toAngle) {
        int diff = wrapAngle360(toAngle) - wrapAngle360(fromAngle);
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    int rawRadarToDisplayAngle(int rawAngle) {
        return wrapAngle360(rawAngle);
    }

    int turretForwardRadarAngle() {
        return wrapAngle360(radarForwardAngle + turretForwardRadarOffsetDeg);
    }

    int azimuthToRadarAngle(int azimuth) {
        azimuth = (int)p.constrain(azimuth, minAzimuth, maxAzimuth);

        int relativeAngle;
        if (azimuth <= homeAzimuth) {
            relativeAngle = (int)p.map(azimuth, minAzimuth, homeAzimuth, -radarAimLeftSpan, 0);
        } else {
            relativeAngle = (int)p.map(azimuth, homeAzimuth, maxAzimuth, 0, radarAimRightSpan);
        }

        if (azimuthDirectionInverted) {
            relativeAngle = -relativeAngle;
        }

        return wrapAngle360(turretForwardRadarAngle() + relativeAngle);
    }

    String currentClockText() {
        return nf(p.hour(), 2) + ":" + nf(p.minute(), 2) + ":" + nf(p.second(), 2);
    }

    void drawRadarPoints() {
        p.noStroke();
        synchronized(pings) {
            for (RadarPing ping : pings) {
                float a = p.radians(ping.angle - 90);
                float rr = p.map(ping.distance, 0, 400, 0, radarRadius);
                float x = p.cos(a) * rr;
                float y = p.sin(a) * rr;

                int alpha = (int)p.map(p.millis() - ping.timestamp, 0, 1200, 255, 0);

                color targetColor = getRangeColor(ping.distance);
                p.fill(p.red(targetColor), p.green(targetColor), p.blue(targetColor), alpha / 2);
                p.ellipse(x, y, 20, 20);
                p.fill(targetColor, alpha);
                p.ellipse(x, y, 8, 8);
            }
        }
    }

    void drawTurretIndicator() {
        int turretRadarAngle = azimuthToRadarAngle(iAzimuth);
        float azimuthRad = p.radians(turretRadarAngle - 90);

        if (isValidDistance(iDistance)) {
            float targetRad = p.radians(iAngle - 90);
            float targetR = p.map(iDistance, 0, 400, 0, radarRadius);
            p.stroke(255, 0, 0, 100);
            p.strokeWeight(1);
            p.line(0, 0, p.cos(targetRad) * targetR, p.sin(targetRad) * targetR);
        }

        if (iState == 3) p.stroke(255, 0, 0, 200);
        else if (iState == 2) p.stroke(255, 255, 0, 180);
        else if (iState == 1) p.stroke(0, 255, 0, 180);
        else p.stroke(0, 150, 255, 120);

        p.strokeWeight(4);
        float sighterLen = radarRadius * 0.85f;
        p.line(0, 0, p.cos(azimuthRad) * sighterLen, p.sin(azimuthRad) * sighterLen);

        if (iState == 3) {
            p.stroke(255, 50, 0, 255);
            p.strokeWeight(8);
            p.line(0, 0, p.cos(azimuthRad) * sighterLen, p.sin(azimuthRad) * sighterLen);
        }

        float tx = p.cos(azimuthRad) * sighterLen;
        float ty = p.sin(azimuthRad) * sighterLen;

        p.fill(255);
        p.noStroke();
        p.ellipse(tx, ty, 10, 10);

        p.fill(0, 180, 255);
        p.textAlign(PConstants.CENTER);
        p.textSize(12);
        p.text("ELV: " + iElevation + "°", tx, ty - 15);
    }

    String getLastTargetTime() {
        return lastTargetClockText;
    }

    void drawSweepIndicator() {
        p.noStroke();
        int trailLength = 25;
        for (int i = 0; i < trailLength; i++) {
            float sweepAngle = currentAngle - i;
            if (sweepAngle < 0) sweepAngle += 360;

            float startAngle = p.radians(sweepAngle - 90);
            float endAngle = p.radians(sweepAngle - 91);

            int alpha = (int)p.map(i, 0, trailLength, 255, 0);
            p.fill(0, 255, 0, alpha / 3);
            p.arc(0, 0, radarRadius * 2, radarRadius * 2, endAngle, startAngle);
        }

        float angleRad = p.radians(currentAngle - 90);
        p.stroke(0, 255, 0, 220);
        p.strokeWeight(4);
        p.line(0, 0, p.cos(angleRad) * radarRadius, p.sin(angleRad) * radarRadius);
    }

    void drawInfoPanels() {
        float x = p.width - 310;
        float y = 100;
        p.pushMatrix();
        p.translate(x, y);
        p.stroke(100);
        p.fill(20, 25, 30, 220);
        p.rect(0, 0, 290, 400, 8);

        p.fill(180, 220, 255);
        p.textAlign(PConstants.LEFT);
        p.textFont(largeFont);
        p.text("معلومات البرج والأهداف", 20, 30);

        p.textFont(font);
        int ly = 65;

        p.fill(200, 255, 200);
        p.text("زاوية عين الرادار: " + iAngle + "°", 20, ly); ly += 30;
        p.text("مسافة الهدف: " + distanceLabel(iDistance), 20, ly); ly += 30;

        color statusColor;
        if (iState == 3) statusColor = p.color(255, 50, 50);
        else if (iState == 2) statusColor = p.color(255, 255, 50);
        else if (iState == 1) statusColor = p.color(50, 255, 50);
        else statusColor = p.color(100, 180, 255);

        p.fill(statusColor);
        p.text("المنظومة: " + alarmStatusText, 20, ly); ly += 30;
        p.text("التنشط: " + threatLevelText, 20, ly); ly += 35;

        p.fill(200, 220, 255);
        p.text("اتجاه برج الإطلاق (Azimuth): " + iAzimuth + "°", 20, ly); ly += 30;
        p.text("ارتفاع الإطلاق (Elevation): " + iElevation + "°", 20, ly); ly += 30;

        float dangerLevel = isValidDistance(iDistance) ? p.map(iDistance, 0, 400, 250, 0) : 0;
        dangerLevel = p.constrain(dangerLevel, 0, 250);
        float colorMix = isValidDistance(iDistance) ? p.map(iDistance, 0, 400, 0, 1) : 1;
        color dangerBarColor = p.lerpColor(p.color(255, 0, 0), p.color(0, 255, 0), colorMix);
        p.fill(dangerBarColor);
        p.noStroke();
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

        p.pushMatrix();
        p.translate(20, 100);
        p.stroke(80);
        p.fill(20, 25, 30, 220);
        p.rect(0, 0, 260, 70, 8);
        p.fill(180, 220, 255);
        p.text("حالة الاتصال:", 15, 25);
        p.fill(isConnected ? p.color(100, 255, 100) : p.color(255, 100, 100));
        p.text((isConnected ? "متصل " : "غير متصل ") + connectionStatus, 15, 50);
        p.popMatrix();
    }

    void drawStatusBarLegacy() {
        p.pushMatrix();
        p.fill(0, 0, 0, 150);
        p.noStroke();
        p.rect(0, p.height - 60, p.width, 60);

        p.fill(0, 255, 0);
        p.textSize(18);
        p.textAlign(PConstants.LEFT);
        p.text("محور العدسة: " + iAngle + "°", 20, p.height - 25);

        p.textAlign(PConstants.CENTER);
        p.text("المسافة: " + distanceLabel(iDistance) + " | توجيه المدفع: " + iAzimuth + "°، الارتفاع: " + iElevation + "°", p.width/2.0f, p.height - 25);

        p.textAlign(PConstants.RIGHT);
        if (iState == 2) {
            p.fill(255, 50, 50);
            p.text("التهديد: " + threatLevelText, p.width - 20, p.height - 25);
        } else {
            p.fill(iState == 1 ? p.color(255, 255, 50) : p.color(0, 255, 0));
            p.text("الاستعداد: " + threatLevelText, p.width - 20, p.height - 25);
        }
        p.popMatrix();
    }

    void handleVisualAlarmLegacy() {
        if (iState == 2) {
            if (p.frameCount % 20 < 10) {
                p.pushMatrix();
                p.translate(radarCenter.x, radarCenter.y);
                p.noStroke();
                p.fill(255, 0, 0, 40);
                p.ellipse(0, 0, radarRadius * 2, radarRadius * 2);
                p.popMatrix();
            }
        }
    }

    void updateStateTextsLegacy() {
        if (iState == 0) {
            threatLevelText = "دورية بحث روتينية";
            alarmStatusText = "مسح (SCAN)";
            targetDetected = false;
        } else if (iState == 1) {
            threatLevelText = "تم رصد هدف، متابعة التوجيه...";
            alarmStatusText = "تتبع (TRACK)";
            if (!targetDetected && isValidDistance(iDistance)) {
                lastTargetClockText = currentClockText();
            }
            targetDetected = true;
        } else if (iState == 2) {
            threatLevelText = "تم قفل الهدف ومحاكاة التوجيه";
            alarmStatusText = "قفل (LOCK)";
            if (!targetDetected && isValidDistance(iDistance)) {
                lastTargetClockText = currentClockText();
            }
            targetDetected = true;
        }
    }

    String distanceLabel(int value) {
        if (isValidDistance(value)) return value + " سم";
        if (value == 0) return "منطقة عمياء <= 20 سم";
        if (value < 0) return "لا يوجد صدى";
        if (value > 400) return "خارج المدى";
        return "---";
    }

    void drawStatusBar() {
        p.pushMatrix();
        p.fill(0, 0, 0, 150);
        p.noStroke();
        p.rect(0, p.height - 60, p.width, 60);

        p.fill(0, 255, 0);
        p.textSize(18);
        p.textAlign(PConstants.LEFT);
        p.text("ظ…ط­ظˆط± ط§ظ„ط¹ط¯ط³ط©: " + iAngle + "آ°", 20, p.height - 25);

        p.textAlign(PConstants.CENTER);
        p.text("ط§ظ„ظ…ط³ط§ظپط©: " + distanceLabel(iDistance) + " | طھظˆط¬ظٹظ‡ ط§ظ„ظ…ط¯ظپط¹: " + iAzimuth + "آ°طŒ ط§ظ„ط§ط±طھظپط§ط¹: " + iElevation + "آ°", p.width/2.0f, p.height - 25);

        p.textAlign(PConstants.RIGHT);
        if (iState == 3) {
            p.fill(255, 50, 50);
            p.text("ط§ظ„طھظ‡ط¯ظٹط¯: " + threatLevelText, p.width - 20, p.height - 25);
        } else {
            if (iState == 2) p.fill(255, 255, 50);
            else if (iState == 1) p.fill(0, 255, 0);
            else p.fill(100, 180, 255);
            p.text("ط§ظ„ط§ط³طھط¹ط¯ط§ط¯: " + threatLevelText, p.width - 20, p.height - 25);
        }
        p.popMatrix();
    }

    void handleVisualAlarm() {
        if (iState == 3) {
            if (p.frameCount % 20 < 10) {
                p.pushMatrix();
                p.translate(radarCenter.x, radarCenter.y);
                p.noStroke();
                p.fill(255, 0, 0, 40);
                p.ellipse(0, 0, radarRadius * 2, radarRadius * 2);
                p.popMatrix();
            }
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
            if (!targetDetected && isValidDistance(iDistance)) {
                lastTargetClockText = currentClockText();
            }
            targetDetected = true;
        } else if (iState == 2) {
            threatLevelText = "تم رصد هدف متوسط المدى (من 151 إلى 250 سم)";
            alarmStatusText = "إنذار متوسط";
            if (!targetDetected && isValidDistance(iDistance)) {
                lastTargetClockText = currentClockText();
            }
            targetDetected = true;
        } else if (iState == 3) {
            threatLevelText = "تم رصد هدف خطر مع تتبع وتصويب (من 21 إلى 150 سم)";
            alarmStatusText = "خطر مع تتبع";
            if (!targetDetected && isValidDistance(iDistance)) {
                lastTargetClockText = currentClockText();
            }
            targetDetected = true;
        }
    }

    void processSerialData(String data) {
        if (data == null || data.trim().isEmpty()) return;
        if (data.startsWith("#")) return;

        String[] parts = data.trim().split(",");
        if (parts.length < 5) {
            return;
        }

        try {
            int rawAngle = p.parseInt(parts[0]);
            iAngle = rawRadarToDisplayAngle(rawAngle);
            iDistance = p.parseInt(parts[1]);
            iState = p.parseInt(parts[2]);
            iAzimuth = p.parseInt(parts[3]);
            iElevation = p.parseInt(parts[4]);

            if (iAngle <= 2 || iAngle >= 358) {
                currentAngle = 0;
            } else {
                float angleDelta = iAngle - currentAngle;
                if (angleDelta > 180) angleDelta -= 360;
                if (angleDelta < -180) angleDelta += 360;
                currentAngle += angleDelta * 0.45f;
                if (currentAngle < 0) currentAngle += 360;
                if (currentAngle >= 360) currentAngle -= 360;
            }

            if (isValidDistance(iDistance)) {
                currentDistance = p.lerp(currentDistance, iDistance, 0.3f);
            } else {
                currentDistance = p.lerp(currentDistance, 0, 0.15f);
            }

            updateStateTexts();

            if (isValidDistance(iDistance)) {
                long nowMs = p.millis();
                float plottedAngleDelta = abs(iAngle - lastPlottedAngle);
                if (plottedAngleDelta > 180) plottedAngleDelta = 360 - plottedAngleDelta;
                float plottedDistanceDelta = abs(iDistance - lastPlottedDistance);

                if (lastPlottedAngle < -360 ||
                    plottedAngleDelta >= 4 ||
                    plottedDistanceDelta >= 10 ||
                    nowMs - lastPlottedTime >= 250) {
                    synchronized(pings) {
                        pings.add(new RadarPing(iAngle, iDistance));
                    }
                    lastPlottedAngle = iAngle;
                    lastPlottedDistance = iDistance;
                    lastPlottedTime = nowMs;
                }
            }
        } catch (Exception e) {
            connectionStatus = "خطأ تحليل بيانات";
        }
    }

    void handleSerialEvent(Serial port) {
        try {
            String data = port.readStringUntil('\n');
            if (data != null) {
                processSerialData(data.trim());
            }
        } catch (Exception e) {
            isConnected = false;
            connectionStatus = "انقطع الاتصال";
        }
    }

    void handleKeyPress() {
        if (p.key == 'c' || p.key == 'C') {
            synchronized(pings) {
                pings.clear();
            }
        } else if (p.key == 'r' || p.key == 'R') {
            setupSerial();
        } else if (p.key == 'p' || p.key == 'P') {
            printAvailablePorts();
        } else if (p.key >= '0' && p.key <= '9') {
            if (p.key == '0') {
                preferredPortHint = "COM10";
            } else {
                preferredPortHint = "COM" + p.key;
            }
            println("Preferred serial port hint set to: " + preferredPortHint);
            setupSerial();
        }
    }
}
