/*
  01_led_buzzer_test
  Isolated acceptance test for:
  - Status LED on D3
  - Buzzer on A5
*/

#define LED_STATUS 3
#define BUZZER_PIN A5

void printIntro() {
  Serial.println(F("START: 01_led_buzzer_test"));
  Serial.println(F("PART: LED + Buzzer"));
  Serial.println(F("PINS: LED_STATUS=D3, BUZZER_PIN=A5"));
  Serial.println(F("POWER: USB is enough for this test"));
  Serial.println(F("PASS CHECK: LED must blink clearly and buzzer must play short and long patterns"));
  Serial.println(F("FAIL SYMPTOM: weak sound, stuck HIGH, random reset, no LED response"));
  Serial.println();
}

void flashLed(int onMs, int offMs, int cycles) {
  for (int i = 0; i < cycles; i++) {
    digitalWrite(LED_STATUS, HIGH);
    delay(onMs);
    digitalWrite(LED_STATUS, LOW);
    delay(offMs);
  }
}

void beepPattern(int frequency, int onMs, int offMs, int cycles) {
  for (int i = 0; i < cycles; i++) {
    tone(BUZZER_PIN, frequency);
    delay(onMs);
    noTone(BUZZER_PIN);
    delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_STATUS, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_STATUS, LOW);
  noTone(BUZZER_PIN);

  delay(250);
  printIntro();
}

void loop() {
  Serial.println(F("RUNNING: slow LED blink phase"));
  flashLed(400, 400, 4);
  Serial.println(F("PASS CHECK: LED slow blink should be clear and stable"));
  delay(500);

  Serial.println(F("RUNNING: fast LED blink phase"));
  flashLed(100, 100, 8);
  Serial.println(F("PASS CHECK: LED fast blink should remain clean without stuck states"));
  delay(500);

  Serial.println(F("RUNNING: short buzzer pattern"));
  beepPattern(2500, 120, 120, 4);
  Serial.println(F("PASS CHECK: you should hear 4 short beeps"));
  delay(500);

  Serial.println(F("RUNNING: long buzzer pattern"));
  beepPattern(1800, 450, 180, 2);
  Serial.println(F("PASS CHECK: you should hear 2 long beeps"));
  delay(500);

  Serial.println(F("RUNNING: combined LED + buzzer pattern"));
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_STATUS, HIGH);
    tone(BUZZER_PIN, 3000);
    delay(150);
    digitalWrite(LED_STATUS, LOW);
    noTone(BUZZER_PIN);
    delay(150);
  }
  Serial.println(F("PASS CHECK: LED and buzzer should stay synchronized"));
  Serial.println(F("CYCLE COMPLETE"));
  Serial.println();

  delay(1500);
}
