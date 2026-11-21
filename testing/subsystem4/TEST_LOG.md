# Subsystem 4 — test log

**Subsystem:** IMU motion, terrain, hump-height and pickup safety  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = `S4_SUMMARY_MAX` | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | Legacy monitor LED pause/resume test | IF-05 (legacy example) | manual | Retained because test IDs are never deleted/reused. Run only if legacy monitor behaviour is still required. | Legacy behaviour observed if applicable |
| S4-IMU01 | IMU initialisation | Buddy4 IMU | auto | `subsystem4_test.c` | `[TEST] S4-IMU01 PASS` |
| S4-IMU02 | IMU calibration | Buddy4 IMU | auto | Keep robot/IMU flat and run test image. | `[TEST] S4-IMU02 PASS` |
| S4-IMU03 | Calibrated accelerometer XYZ read | Buddy4 IMU | auto | `subsystem4_test.c` | `[TEST] S4-IMU03 PASS` |
| S4-IMU04 | Filtered accelerometer XYZ read | Buddy4 IMU | auto | `subsystem4_test.c` | `[TEST] S4-IMU04 PASS` |
| S4-IMU05 | Flat baseline capture | Terrain baseline | auto | Keep robot/IMU flat during baseline capture. | `[TEST] S4-IMU05 PASS` |
| S4-IMU06 | Flat baseline classification | Terrain detection | auto | `subsystem4_test.c` | `[TEST] S4-IMU06 PASS` |
| S4-IMU07 | Uphill threshold classification | Terrain detection | auto | `subsystem4_test.c` | `[TEST] S4-IMU07 PASS` |
| S4-IMU08 | Peak threshold classification | Terrain detection | auto | `subsystem4_test.c` | `[TEST] S4-IMU08 PASS` |
| S4-IMU09 | Downhill threshold classification | Terrain detection | auto | `subsystem4_test.c` | `[TEST] S4-IMU09 PASS` |
| S4-U03 | IMU status contains only terrain + fault bits | Buddy4 status interface | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | Motion status contains only documented motion bits | Buddy4 motion interface | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | Compact motion mapping: bit0 acceleration, bit1 turning, bit2 lift | Buddy4 motion interface | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | Current/highest hump-height getters return consistent non-negative values | Buddy4 height interface | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-M01 | Full terrain sequence | Hump / terrain detection | manual | Run robot over hump while saving serial output. Observe `FLAT -> UPHILL -> PEAK -> DOWNHILL -> FLAT`. | All terrain states occur in order and return to FLAT |
| S4-M02 | Hump-height measurement | Demo: measure hump height | manual | Measure physical hump with ruler; run robot over it; save serial output and compare reported height with reference measurement. | Hump is detected and a plausible calibrated height is reported |
| S4-M03 | Highest-hump tracking | Mission: report highest hump | manual | Run over two or more humps where practical; save reported current/highest values. | Highest value never decreases and represents the largest measured hump |
| S4-M04 | Acceleration detection | Motion classification | manual | Move/drive robot while saving serial output, then leave stationary. | Acceleration bit/event sets during motion change and clears when stationary |
| S4-M05 | Turning detection | Motion / turn-rate reporting | manual | Turn robot while saving serial output, then stop turning. | Turning bit/event sets; turn-rate is reported; bit clears after turn ends |
| S4-M06 | Pickup/lift detection and safety-stop request | Updated pickup requirement | manual | After `PICKUP SAFETY ARMED`, lift robot while saving serial output. | `LIFTED` is reported, motion bit2 sets, and `motion_stop()` is called and Buddy2 receives `CMD_STOP` |
| S4-M07 | Physical motor stop after pickup | Demo: stop motors when car is picked up | manual | With wheels physically running, record video + serial; lift robot after pickup safety is armed. | Both physical wheels stop promptly after pickup |
| S4-M08 | IMU fault reporting | IMU fault/status | manual | Reproduce an IMU read/init failure only when safe and practical; save serial output. | IMU fault bit is set and fault is reported |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;  
**manual** = a person observes the result using console output, measurement, or video.

### Evidence guidance

- Save automated/test-image console output under `testing/subsystem4/logs/`.
- Save terrain, hump-height, acceleration, turning and pickup serial output under `testing/subsystem4/logs/`.
- For **S4-M07**, use a short video as the primary physical evidence because console state alone cannot prove that the wheels actually stopped. Add the unlisted video link to `videos/README.md` and reference it in the result row.
- Do not record a PASS until the test has actually been run on the stated commit.
- Latest S4-M07 status (21 Nov 2026): **PASS in two manual pickup tests.** Buddy2 processed the stop command and Raisa confirmed the wheels physically stopped. No video was supplied; earlier failed attempts remain part of the testing history.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-21 | S4-M06 | `7c2a723` | Raisa | PASS | COM9 serial, pickup test 1 | `LIFTED tilt=21 y_change=2528`; motion status `0x05`; `SAFETY STOP REQUEST result=2`; `[S2] STOP RECEIVED`. |
| 2026-11-21 | S4-M07 | `7c2a723` | Raisa | PASS | COM9 serial, pickup test 1; tester confirmation | `[S2] BRAKE APPLIED`; Raisa confirmed wheels stopped. No video supplied; stop latency not measured. |
| 2026-11-21 | S4-M06 | `7c2a723` | Raisa | PASS | COM9 serial, pickup test 2 | `LIFTED tilt=20 y_change=3514`; motion status `0x07`; `SAFETY STOP REQUEST result=2`; `[S2] STOP RECEIVED`. |
| 2026-11-21 | S4-M07 | `7c2a723` | Raisa | PASS | COM9 serial, pickup test 2; tester confirmation | `[S2] BRAKE APPLIED`; Raisa confirmed wheels stopped. No video supplied; stop latency not measured. |
