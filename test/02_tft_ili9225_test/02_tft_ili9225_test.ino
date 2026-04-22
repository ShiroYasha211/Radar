/*
  02_tft_ili9225_test
  Isolated acceptance test for the local TFT display.
*/

#include <SPI.h>
#include <TFT_22_ILI9225.h>

#define TFT_RST 7
#define TFT_RS 6
#define TFT_CS 10

#define COLOR_BLACK  0x0000
#define COLOR_BLUE   0x001F
#define COLOR_RED    0xF800
#define COLOR_GREEN  0x07E0
#define COLOR_CYAN   0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW 0xFFE0
#define COLOR_WHITE  0xFFFF
#define COLOR_GRAY   0x8410

TFT_22_ILI9225 tft(TFT_RST, TFT_RS, TFT_CS, 0);

void printIntro() {
  Serial.println(F("START: 02_tft_ili9225_test"));
  Serial.println(F("PART: TFT 2.2 ILI9225"));
  Serial.println(F("PINS: RST=D7, RS=D6, CS=D10, MOSI=D11, MISO=D12, SCK=D13"));
  Serial.println(F("POWER: USB is usually enough for this display test"));
  Serial.println(F("PASS CHECK: no black screen, no corrupted colors, no heavy flicker"));
  Serial.println(F("FAIL SYMPTOM: blank display, random pixels, unstable refresh, wrong colors"));
  Serial.println();
}

void drawColorBars() {
  tft.clear();
  tft.fillRectangle(0, 0, 175, 29, COLOR_RED);
  tft.fillRectangle(0, 30, 175, 59, COLOR_GREEN);
  tft.fillRectangle(0, 60, 175, 89, COLOR_BLUE);
  tft.fillRectangle(0, 90, 175, 119, COLOR_YELLOW);
  tft.fillRectangle(0, 120, 175, 149, COLOR_CYAN);
  tft.fillRectangle(0, 150, 175, 176, COLOR_WHITE);
  tft.drawText(8, 8, "COLOR BARS", COLOR_BLACK);
}

void drawGeometry() {
  tft.clear();
  tft.drawRectangle(5, 5, 170, 171, COLOR_WHITE);
  tft.drawLine(0, 0, 175, 175, COLOR_RED);
  tft.drawLine(175, 0, 0, 175, COLOR_GREEN);
  tft.drawCircle(88, 88, 50, COLOR_CYAN);
  tft.fillCircle(88, 88, 8, COLOR_YELLOW);
  tft.drawText(45, 155, "LINES/CIRCLES", COLOR_WHITE);
}

void drawTextPanel() {
  tft.clear();
  tft.setFont(Terminal6x8);
  tft.drawText(4, 8, "TFT ILI9225 TEST", COLOR_GREEN);
  tft.drawText(4, 26, "UNO pins in use:", COLOR_WHITE);
  tft.drawText(4, 42, "RST D7", COLOR_YELLOW);
  tft.drawText(4, 54, "RS  D6", COLOR_YELLOW);
  tft.drawText(4, 66, "CS  D10", COLOR_YELLOW);
  tft.drawText(4, 78, "MOSI D11", COLOR_YELLOW);
  tft.drawText(4, 90, "MISO D12", COLOR_YELLOW);
  tft.drawText(4, 102, "SCK  D13", COLOR_YELLOW);
  tft.drawText(4, 122, "PASS CHECK:", COLOR_CYAN);
  tft.drawText(4, 134, "Text must be sharp", COLOR_CYAN);
  tft.drawText(4, 146, "No missing lines", COLOR_CYAN);
  tft.drawText(4, 158, "No color inversion", COLOR_CYAN);
}

void drawStableHoldScreen() {
  tft.clear();
  tft.drawRectangle(2, 2, 173, 173, COLOR_GRAY);
  for (int i = 20; i < 160; i += 20) {
    tft.drawLine(8, i, 167, i, COLOR_BLUE);
  }
  tft.drawText(26, 30, "STABLE HOLD", COLOR_WHITE);
  tft.drawText(16, 54, "Observe for flicker", COLOR_WHITE);
  tft.drawText(18, 78, "and color stability", COLOR_WHITE);
  tft.drawText(24, 120, "HOLD 3 SECONDS", COLOR_GREEN);
}

void setup() {
  Serial.begin(115200);
  delay(250);
  printIntro();

  tft.begin();
  tft.setOrientation(1);
  tft.clear();
}

void loop() {
  Serial.println(F("RUNNING: color bars"));
  drawColorBars();
  delay(2500);

  Serial.println(F("RUNNING: geometry screen"));
  drawGeometry();
  delay(2500);

  Serial.println(F("RUNNING: text and pin map"));
  drawTextPanel();
  delay(3000);

  Serial.println(F("RUNNING: stable hold screen"));
  drawStableHoldScreen();
  Serial.println(F("PASS CHECK: display should stay stable during hold"));
  Serial.println(F("CYCLE COMPLETE"));
  Serial.println();
  delay(3000);
}
