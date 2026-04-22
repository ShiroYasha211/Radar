# Test Results Template

Use one row per hardware test run.

| Test ID | Part | Final Pins | Power Source | Result | Notes / Limits |
| --- | --- | --- | --- | --- | --- |
| 01 | LED + Buzzer | D3, A5 | USB | PASS / PASS WITH LIMITS / FAIL | |
| 02 | TFT ILI9225 | RST D7, RS D6, CS D10, SPI | USB | PASS / PASS WITH LIMITS / FAIL | |
| 03 | HC-SR04 | TRIG D8, ECHO A4 | USB | PASS / PASS WITH LIMITS / FAIL | |
| 04 | Servo Azimuth | D9 | External + shared GND | PASS / PASS WITH LIMITS / FAIL | |
| 05 | Servo Elevation | D5 | External + shared GND | PASS / PASS WITH LIMITS / FAIL | |
| 06 | Stepper 28BYJ-48 | A0, A1, A2, A3 | External + shared GND | PASS / PASS WITH LIMITS / FAIL | |
| 07 | DC Motor + L298N | IN1 D4, IN2 D2 | External + shared GND | PASS / PASS WITH LIMITS / FAIL | |
| 08 | Serial + Processing | USB serial | USB | PASS / PASS WITH LIMITS / FAIL | |

Suggested notes:
- Smoothness
- Noise or vibration
- Safe angle limits
- Direction correctness
- Range limits
- Any wiring observations
