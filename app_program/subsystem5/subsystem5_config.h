#ifndef SUBSYSTEM5_CONFIG_H
#define SUBSYSTEM5_CONFIG_H

/* HC-SR04 Ultrasonic Sensor */
#define S5_TRIG_PIN  4
#define S5_ECHO_PIN  5

/* Servo Motor */
#define S5_SERVO_PIN 8

/* Scanning Configuration */
#define S5_SCAN_COUNT 5
#define S5_DISTANCE_LIMIT 100.0f

#define S5_SCAN_DELAY_MS 100
#define S5_ECHO_TIMEOUT_US 30000

#endif
