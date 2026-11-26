# Subsystem 1 — Test Log

*Subsystem:* Wi-Fi / MQTT Communication & Telemetry  
*Student ID:* 2503784
*Code:* app_program/subsystem1/

Subsystem 1 provides the communication layer for the robot. Testing covers the Buddy 1 command parser and message buffer, Wi-Fi/MQTT connectivity, MQTT telemetry transmission, heartbeat communication, and telemetry integration with the other subsystems.

## Test Cases

| *ID* | *What is tested* | *Requirement / interface* | *Method* | *Steps / how* | *Expected result* |
|---|---|---|---|---|---|
| S1-U02 | LED task sleeps between ticks | Subsystem 1 task behaviour | auto | Run subsystem1_test.c | [TEST] S1-U02 PASS |
| S1-U03 | START command parsing | buddy1_parse_command() | auto | Pass START to command parser | START command accepted |
| S1-U04 | STOP command parsing | buddy1_parse_command() | auto | Pass STOP to command parser | STOP command accepted |
| S1-U05 | LEFT command parsing | buddy1_parse_command() | auto | Pass LEFT to command parser | LEFT command accepted |
| S1-U06 | Invalid MQTT command rejection | buddy1_parse_command() | auto | Pass an unsupported command | Invalid command rejected |
| S1-U07 | Buddy 1 command message buffer creation | mbf_buddy1_cmd | auto | Start Subsystem 1 test firmware | Command buffer successfully created |
| S1-U08 | Command buffer send/receive | buddy1_queue_command() / message buffer | auto | Queue a command and receive it from the buffer | Sent command is received correctly |
| S1-U09 | Public command receive API | buddy1_receive_command() | auto | Receive queued command using public API | Correct command returned |
| S1-I01 | Wi-Fi connection | CYW43 Wi-Fi service | manual | Flash normal image and connect Pico W to configured hotspot | Wi-Fi joins AP and obtains IP address |
| S1-I02 | MQTT broker connection and subscription | mqtt_utk.c | manual | Start Mosquitto broker and robot | MQTT connects and subscribes to buddy1/test |
| S1-I03 | MQTT telemetry TX | buddy1_send_packet() → TX message buffer → MQTT service | manual | Run robot while subscribed to buddy1/test | Telemetry packets are transmitted through MQTT |
| S1-I04 | MQTT heartbeat communication | MQTT service | manual | Keep MQTT connection active and observe serial output | Heartbeat is transmitted/received |
| S1-I05 | Buddy 4 hump telemetry integration | BUDDY1_TLM_HUMP | manual | Drive robot over hump while MQTT is connected | Hump status and height telemetry are transmitted |
| S1-I06 | Buddy 4 pickup/collision telemetry integration | BUDDY1_TLM_HUMP status | manual | Pick up robot after pickup safety is armed | Lift is detected, motors stop, and updated telemetry is transmitted |
| S1-I07 | Buddy 2 motor/distance telemetry integration | BUDDY1_TLM_MOTOR, BUDDY1_TLM_DISTANCE | manual | Drive robot while external MQTT decoder is running | Motor speed, encoder and distance values update on MQTT client |
| S1-I08 | Buddy 3 line telemetry integration | BUDDY1_TLM_LINE | manual | Move robot across line while external MQTT decoder is running | Left/centre/right IR states update on MQTT client |
| S1-I09 | Buddy 5 obstacle telemetry integration | BUDDY1_TLM_OBSTACLE | manual | Place obstacle in robot path while external MQTT decoder is running | Obstacle detection and measurement values update on MQTT client |

Method: *auto* = automated TEST_CHECK result printed to the console; *manual* = result verified using serial output, physical robot behaviour, or external MQTT subscriber/decoder.

## Results

Add a new row for every run. Never edit or delete old rows.

| *Date* | *Test ID* | *Commit* | *Tester* | *Result* | *Log / evidence* | *Notes* |
|---|---|---|---|---|---|---|
| 2026-11-24 | S1-U02 | — | Tze Han | PASS | Serial log | LED task sleeps between ticks |
| 2026-11-24 | S1-U03 | — | Tze Han | PASS | Serial log | START command parsed |
| 2026-11-24 | S1-U04 | — | Tze Han | PASS | Serial log | STOP command parsed |
| 2026-11-24 | S1-U05 | — | Tze Han | PASS | Serial log | LEFT command parsed |
| 2026-11-24 | S1-U06 | — | Tze Han | PASS | Serial log | Invalid command rejected |
| 2026-11-24 | S1-U07 | — | Tze Han | PASS | Serial log | Buddy 1 command buffer created |
| 2026-11-24 | S1-U08 | — | Tze Han | PASS | Serial log | Buddy 1 command buffer send/receive |
| 2026-11-24 | S1-U09 | — | Tze Han | PASS | Serial log | Buddy 1 public receive API |
| 2026-11-24 | S1-I01 | — | Team | PASS | Serial log | Wi-Fi obtained IP 10.97.208.18 |
| 2026-11-24 | S1-I02 | — | Team | PASS | Serial log | MQTT connected and subscribed to buddy1/test |
| 2026-11-24 | S1-I03 | — | Team | PASS | Serial log | Repeated mqtt: telemetry TX 4 bytes observed |
| 2026-11-24 | S1-I04 | — | Team | PASS | Serial log | mqtt: RX [buddy1/test] "hb 1" observed |
| 2026-11-24 | S1-I05 | — | Team | PASS | Serial log | Hump reached 8 mm, highest recorded 10 mm, followed by MQTT telemetry TX |
| 2026-11-24 | S1-I06 | — | Team | PASS | Serial log | MOTION EVENT = LIFTED, followed by STOP RECEIVED, BRAKE APPLIED and MQTT telemetry |
| 2026-11-26 | S1-I07 | — | Team | PASS | External MQTT decoder | Live motor speed, encoder counts and distance received |
| 2026-11-26 | S1-I08 | — | Team | PASS | External MQTT decoder | Live L/C/R line-sensor states received |
| 2026-11-26 | S1-I09 | — | Team | PASS | External MQTT decoder | Live obstacle distance, width, scan span and bypass distance received |

## Serial Evidence

### Buddy 1 Unit Tests

text
[TEST] S1-U02 PASS  LED task sleeps between ticks
[TEST] S1-U03 PASS  START command parsed
[TEST] S1-U04 PASS  STOP command parsed
[TEST] S1-U05 PASS  LEFT command parsed
[TEST] S1-U06 PASS  Invalid command rejected
[TEST] S1-U07 PASS  Buddy 1 command buffer created
[TEST] S1-U08 PASS  Buddy 1 command buffer send/receive
[TEST] S1-U09 PASS  Buddy 1 public receive API
[TEST] subsystem1 SUMMARY 9 passed, 0 failed


### Wi-Fi and MQTT

text
wifi: link status -> joined AP, no IP yet (2)
wifi: link up, ip=10.97.208.18

mqtt: connected to 10.97.208.187
mqtt: telemetry TX 4 bytes
mqtt: subscribed to buddy1/test
mqtt: telemetry TX 4 bytes
mqtt: telemetry TX 4 bytes
mqtt: telemetry TX 4 bytes

mqtt: RX [buddy1/test] "...."
mqtt: telemetry TX 4 bytes


### MQTT Heartbeat

text
mqtt: RX [buddy1/test] "...."
mqtt: telemetry TX 4 bytes
mqtt: RX [buddy1/test] "hb 1"
mqtt: RX [buddy1/test] "...."
mqtt: telemetry TX 4 bytes


### Hump Telemetry

text
imu: STATUS CHANGE = 0x01 terrain=1 fault=0
imu: TERRAIN = UPHILL baseline_y=307 xyz=6112,3744,22617

imu: MOTION STATUS = 0x03 accelerating=1 turning=1 lifted=0
imu: MOTION EVENT = TURNING
imu: TURN RATE = 40 deg/s

mqtt: telemetry TX 4 bytes
mqtt: RX [buddy1/test] "...."

imu: HUMP HEIGHT = 8 mm highest=10 mm
imu: STATUS CHANGE = 0x02 terrain=2 fault=0
imu: TERRAIN = PEAK

mqtt: telemetry TX 4 bytes
mqtt: RX [buddy1/test] "...."


### Pickup / Collision Safety and MQTT

text
imu: MOTION STATUS = 0x07 accelerating=1 turning=1 lifted=1
imu: MOTION EVENT = LIFTED tilt=25 y_change=4396
imu: PICKUP STOP REQUEST result=678

imu: MOTION STATUS = 0x05 accelerating=1 turning=0 lifted=1
imu: MOTION EVENT = TURN END

[S2] STOP RECEIVED
[S2] BRAKE APPLIED

mqtt: telemetry TX 4 bytes
mqtt: RX [buddy1/test] ".+.."

imu: MOTION STATUS = 0x04 accelerating=0 turning=0 lifted=1
imu: MOTION = STATIONARY
