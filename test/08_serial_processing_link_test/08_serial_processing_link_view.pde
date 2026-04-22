import processing.serial.*;

Serial activePort;

String[] portList = {};
int portIndex = 0;

String preferredPortHint = "";
String connectionStatus = "DISCONNECTED";
String lastRawPacket = "No packet yet";

int angleValue = 0;
int distanceValue = 0;
int stateValue = 0;
int azimuthValue = 0;
int elevationValue = 0;

void setup() {
  size(900, 520);
  frameRate(30);
  textFont(createFont("Arial", 18));
  refreshPortList();
  connectToSelectedPort();
}

void draw() {
  background(18, 24, 30);

  fill(230);
  textSize(28);
  text("Serial Link Test Viewer", 30, 40);

  textSize(16);
  fill(120, 220, 255);
  text("Status: " + connectionStatus, 30, 78);
  text("Controls: R reconnect | N next port", 30, 104);
  text("Preferred hint: '" + preferredPortHint + "'", 30, 130);

  fill(220);
  text("Available ports:", 30, 178);
  for (int i = 0; i < portList.length; i++) {
    fill(i == portIndex ? color(255, 220, 80) : color(180));
    text((i == portIndex ? "> " : "  ") + portList[i], 30, 206 + i * 22);
  }

  drawValueBox(420, 60, 190, 90, "ANGLE", str(angleValue));
  drawValueBox(640, 60, 190, 90, "DISTANCE", str(distanceValue) + " cm");
  drawValueBox(420, 180, 190, 90, "STATE", stateLabel(stateValue));
  drawValueBox(640, 180, 190, 90, "AZIMUTH", str(azimuthValue) + " deg");
  drawValueBox(530, 300, 190, 90, "ELEVATION", str(elevationValue) + " deg");

  fill(220);
  text("Last raw packet:", 30, 380);
  fill(160, 255, 160);
  text(lastRawPacket, 30, 410);

  fill(180);
  text("Expected packet format: angle,distance,state,azimuth,elevation", 30, 460);
}

void drawValueBox(int x, int y, int w, int h, String title, String value) {
  noStroke();
  fill(34, 44, 54);
  rect(x, y, w, h, 10);

  fill(150, 210, 255);
  textSize(16);
  text(title, x + 18, y + 28);

  fill(255);
  textSize(28);
  text(value, x + 18, y + 64);
}

String stateLabel(int value) {
  if (value == 0) return "SCAN";
  if (value == 1) return "TRACK";
  if (value == 2) return "FIRE";
  return "UNKNOWN";
}

void refreshPortList() {
  portList = Serial.list();
  if (portList == null) {
    portList = new String[0];
  }

  if (preferredPortHint.length() > 0) {
    for (int i = 0; i < portList.length; i++) {
      if (portList[i].indexOf(preferredPortHint) >= 0) {
        portIndex = i;
        return;
      }
    }
  }

  if (portIndex >= portList.length) {
    portIndex = 0;
  }
}

void connectToSelectedPort() {
  closeCurrentPort();
  refreshPortList();

  if (portList.length == 0) {
    connectionStatus = "NO SERIAL PORTS FOUND";
    return;
  }

  try {
    activePort = new Serial(this, portList[portIndex], 115200);
    activePort.bufferUntil('\n');
    connectionStatus = "CONNECTED TO " + portList[portIndex];
  } catch (Exception e) {
    activePort = null;
    connectionStatus = "FAILED TO CONNECT: " + portList[portIndex];
  }
}

void closeCurrentPort() {
  if (activePort != null) {
    activePort.stop();
    activePort = null;
  }
}

void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if (line == null) {
    return;
  }

  line = trim(line);
  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("#")) {
    lastRawPacket = line;
    return;
  }

  String[] parts = split(line, ',');
  if (parts.length != 5) {
    lastRawPacket = "BAD PACKET: " + line;
    return;
  }

  try {
    angleValue = parseInt(parts[0]);
    distanceValue = parseInt(parts[1]);
    stateValue = parseInt(parts[2]);
    azimuthValue = parseInt(parts[3]);
    elevationValue = parseInt(parts[4]);
    lastRawPacket = line;
  } catch (Exception e) {
    lastRawPacket = "PARSE ERROR: " + line;
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    connectToSelectedPort();
  } else if (key == 'n' || key == 'N') {
    if (portList.length > 0) {
      portIndex = (portIndex + 1) % portList.length;
      connectToSelectedPort();
    }
  }
}
