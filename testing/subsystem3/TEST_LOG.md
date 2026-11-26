# Subsystem 3 — Test Log

*Subsystem:* IR Line Following & Barcode Decoding  
*Student ID:* 2503123
*Code:* app_program/subsystem3/

Subsystem 3 uses the three IR sensors for line detection, line following, junction detection and barcode scanning. It also provides line and barcode information to the other subsystems for navigation and telemetry.

## Test Cases

| *ID* | *What is tested* | *Requirement / interface* | *Method* | *Steps / how* | *Expected result* |
|---|---|---|---|---|---|
| S3-U01 | Received sample range is valid | Subsystem 3 test interface | auto | Run subsystem3_test.c | Minimum ≤ maximum and values remain within 0–100 |
| S3-I01 | Data flows from Subsystem 2 to Subsystem 3 | Subsystem integration | auto | Run integration test | Subsystem 3 successfully receives data |
| S3-M01 | Left IR sensor detects line | 3× IR sensor input | manual | Move line under left sensor | Left sensor changes state |
| S3-M02 | Centre IR sensor detects line | 3× IR sensor input | manual | Move line under centre sensor | Centre sensor changes state |
| S3-M03 | Right IR sensor detects line | 3× IR sensor input | manual | Move line under right sensor | Right sensor changes state |
| S3-M04 | Line-following steering responds to IR position | Line-following algorithm | manual | Run robot over line and observe IR/steering output | Steering response changes according to detected line position |
| S3-M05 | Barcode pass is detected and segments are recorded | Barcode detection | manual | Drive barcode past barcode/IR sensor | Barcode pass ends and captured edges/segments are reported |
| S3-M06 | Valid Code39 barcode is decoded | Barcode decoder | manual | Drive a supported barcode through sensor | Supported barcode character is decoded correctly |
| S3-M07 | Decoded barcode generates navigation action | Navigation command interface | manual | Scan supported A/B/C/D barcode | Correct LEFT/FORWARD/RIGHT/UTURN action is generated |
| S3-M08 | Brief line loss can be handled | Line-following fault handling | manual | Introduce a short gap in the line | Robot continues/reacquires line without unsafe behaviour |
| S3-M09 | Extended line loss is handled safely | Line-following fault handling | manual | Remove line for an extended period | Robot does not continue indefinitely using invalid line data |
| S3-M10 | Junction / corner detection | Junction handling | manual | Drive robot into junction/corner | Junction is detected and navigation logic responds |
| S3-I02 | Line status is exposed through public interface | s3_get_line_status() | manual | Run integrated firmware and vary IR states | Public line status changes with L/C/R sensor states |
| S3-I03 | Buddy 3 line status reaches MQTT client | Buddy3 → Buddy1 telemetry | manual | Run external MQTT telemetry decoder while robot follows line | MQTT client displays changing L/C/R states |
| S3-I04 | Barcode telemetry packet reaches MQTT client | s3_get_last_barcode_char(), s3_get_barcode_action() | manual | Run external MQTT decoder | Barcode packet is received and decoded by client |

Method: *auto* = automated test result printed to the console; *manual* = result verified using physical robot behaviour, serial output, or the external MQTT client.

## Results

Add a new row for every run. Never edit or delete old rows.

| *Date* | *Test ID* | *Commit* | *Tester* | *Result* | *Log / evidence* | *Notes* |
|---|---|---|---|---|---|---|
| 2026-11-24 | S3-U01 | — | Raisa | PASS | Serial log | min <= max, both within 0..100 |
| 2026-11-24 | S3-I01 | — | Team | PASS | Serial log | samples flow from subsystem 2 to subsystem 3 |
| 2026-11-24 | S3-M01 | — | Raisa | PASS | Serial [IR-DEBUG] | Left sensor state observed changing |
| 2026-11-24 | S3-M02 | — | Raisa | PASS | Serial [IR-DEBUG] | Centre sensor state observed changing |
| 2026-11-24 | S3-M03 | — | Raisa | PASS | Serial [IR-DEBUG] | Right sensor state observed changing |
| 2026-11-24 | S3-M04 | — | Raisa | PASS | Serial [IR-DEBUG] | IR states and steering output changed during physical testing |
| 2026-11-24 | S3-M05 | — | Raisa | PASS | Serial [BC-PASS] | Barcode pass detected; duration, edges and segment timings recorded |
| — | S3-M06 | — | Raisa | NOT VERIFIED | — | Physical successful Code39 A/B/C/D decode not present in recovered logs |
| — | S3-M07 | — | Raisa | NOT VERIFIED | — | Navigation mapping exists in code, but successful physical barcode decode/action is not proven by recovered logs |
| — | S3-M08 | — | Raisa | NOT VERIFIED | — | Test intended but no conclusive recovered result |
| — | S3-M09 | — | Raisa | NOT VERIFIED | — | Test intended but no conclusive recovered result |
| — | S3-M10 | — | Raisa | NOT VERIFIED | — | No conclusive dedicated junction test recovered |
| 2026-11-26 | S3-I02 | — | Team | PASS | Integrated serial/MQTT test | Public line status changes with physical IR sensor states |
| 2026-11-26 | S3-I03 | — | Team | PASS | External MQTT decoder | Live L/C/R states received and changed while robot moved |
| 2026-11-26 | S3-I04 | — | Team | PASS | External MQTT decoder | Barcode telemetry packet received; latest test remained decoded=NONE action=NONE |

## Serial Evidence

### Automated Subsystem 3 Tests

text id="z5yrcm"
[TEST] INT-01 PASS  samples flow from subsystem 2 to subsystem 3
[TEST] S3-U01 PASS  min <= max, both within 0..100
[TEST] subsystem3 SUMMARY 2 passed, 0 failed


### IR Sensor / Line-Following Evidence

text id="mxn0ad"
[IR-DEBUG] L:0 C:0 R:0 | Steer:0
[IR-DEBUG] L:0 C:0 R:1 | Steer:0
[IR-DEBUG] L:0 C:1 R:1 | Steer:30


Another captured sequence:

text id="q2i93s"
[IR-DEBUG] L:0 C:0 R:0 | Steer:0
[IR-DEBUG] L:1 C:0 R:0 | Steer:-30
[IR-DEBUG] L:0 C:0 R:0 | Steer:0
[IR-DEBUG] L:0 C:0 R:1 | Steer:0


These logs show that the individual IR states change during physical testing and that the line-following logic produces different steering outputs.

### Barcode Pass Detection

text id="xlyp5c"
[BC-PASS] END: dur=1041 ms | edges=4 | bits=0x5
  seg[0]: val=0 held=849730 us
  seg[1]: val=1 held=150316 us
  seg[2]: val=0 held=41150 us
  seg[3]: val=1 held=28386 us (trailing)


Another physical scan:

text id="9wz7cj"
[BC-PASS] END: dur=1069 ms | edges=2 | bits=0x1
  seg[0]: val=0 held=1069878 us
  seg[1]: val=1 held=9942 us (trailing)


This confirms that the barcode tracking logic detects a barcode/pass event and records transition timing. These particular scans did not contain enough valid transitions to prove successful Code39 decoding.

## Public Interfaces Used for Integration

Buddy 3 exposes the current line state and barcode/navigation information through:

text id="pj21c2"
s3_get_line_status()
s3_get_last_barcode_char()
s3_get_barcode_action()


These interfaces allow Buddy 1 to obtain Buddy 3 information without directly accessing the IR/barcode implementation.

## MQTT Integration

During the final integrated test, Buddy 3 line telemetry was received by the external MQTT decoder.

The line packet reports the three sensor states:

text id="7tss2h"
L = left IR sensor
C = centre IR sensor
R = right IR sensor