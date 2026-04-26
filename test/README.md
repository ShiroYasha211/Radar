# Hardware Isolation Test Pack

This folder now contains isolated Arduino hardware tests for the Radar project.

Why the tests are in separate subfolders:
- Arduino IDE treats one sketch folder as one project.
- Keeping each test in its own folder prevents accidental cross-compilation.
- Each test uses the final project pins whenever possible.

Board and power assumptions:
- Board: Arduino UNO
- Power for servo, stepper, and DC motor: external supply
- Always connect external supply GND to Arduino GND
- Do not power high-current parts from USB alone

Recommended test order:
1. `01_led_buzzer_test`
2. `02_tft_ili9225_test`
3. `03_ultrasonic_hcsr04_test`
4. `18_ultrasonic_range_quality_test`
5. `04_servo_azimuth_test`
6. `05_servo_elevation_test`
7. `12_elevation_servo_manual_range_test`
8. `06_stepper_28byj48_test`
9. `07_dc_motor_l298n_test`
10. `13_manual_turret_fire_test`
11. `14_hall_home_switch_test`
12. `15_hall_sensor_debug_test`
13. `16_hall_sensor_universal_probe_test`
14. `17_49e_home_sensor_test`
15. `08_serial_processing_link_test`

Acceptance workflow:
1. Open one sketch folder at a time.
2. Upload the sketch.
3. Read the serial instructions at `115200`.
4. Observe the expected behavior.
5. Record the result in `TEST_RESULTS_TEMPLATE.md`.

Folder map:
- `legacy_combined_test/`: archived combined test, kept only for reference
- `01_led_buzzer_test/`: LED and buzzer only
- `02_tft_ili9225_test/`: local TFT display only
- `03_ultrasonic_hcsr04_test/`: HC-SR04 only
- `18_ultrasonic_range_quality_test/`: ultrasonic sensor real range, stability, and no-echo rate test
- `04_servo_azimuth_test/`: horizontal servo only
- `05_servo_elevation_test/`: vertical servo only
- `12_elevation_servo_manual_range_test/`: manual exact-angle control for azimuth + elevation servos
- `06_stepper_28byj48_test/`: 28BYJ-48 + ULN2003 only
- `07_dc_motor_l298n_test/`: L298N + DC motor only
- `13_manual_turret_fire_test/`: manual turret aiming + launch motor validation
- `14_hall_home_switch_test/`: A3144 Hall effect sensor as radar home/reference switch
- `15_hall_sensor_debug_test/`: deeper D10/Hall/magnet diagnostic isolation test
- `16_hall_sensor_universal_probe_test/`: A0 + 10k universal Hall sensor identification test
- `17_49e_home_sensor_test/`: dedicated calibration test for 49E / 49E513 analog Hall sensor
- `08_serial_processing_link_test/`: serial packet generator + Processing viewer
