# Subsystem 4 — test log

**Subsystem:** IMU motion, terrain, hump-height and pickup safety  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | Message buffer max line = `S4_SUMMARY_MAX` | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | Legacy monitor LED pause/resume test | IF-05 (legacy example) | manual | Retained because test IDs are never deleted/reused. Run only if legacy monitor behaviour is still required. | Legacy behaviour observed if applicable |
| S4-IMU01 | IMU initialisation | Buddy4 IMU | auto | `subsystem4_test.c` | `[TEST] S4-IMU01 PASS` |
| S4-IMU02 | IMU calibration | Buddy4 IMU | auto | Keep robot/IMU flat and run test image. | `[TEST] S4-IMU02 PASS` |
| S4-IMU03 | Calibrated accelerometer XYZ read | Buddy4 IMU | auto | `subsystem4_test.c` | `[TEST] S4-IMU03 PASS` |
| S4-IMU04 | Filtered accelerometer XYZ read | Buddy4 IMU | auto | `subsystem4_test.c` | `[TEST] S4-IMU04 PASS` |
| S4-IMU05 | Flat baseline capture | Terrain baseline | auto | Keep robot/IMU flat during baseline capture. | `[TEST] S4-IMU05 PASS` |
| S4-IMU06 | Flat baseline classified as FLAT by runtime FSM | Terrain detection | manual | Run normal firmware (`TEST=0`), observe baseline and terrain state before hump. | Terrain remains FLAT at baseline |
| S4-IMU07 | Positive-Y uphill transition | Terrain detection | manual | Run normal firmware and observe Y rise above baseline with confirmation samples. | `TERRAIN = UPHILL` |
| S4-IMU08 | Peak transition after Y falls from recorded maximum | Terrain detection | manual | Run normal firmware over hump and observe transition after uphill maximum. | `TERRAIN = PEAK` |
| S4-IMU09 | Downhill transition after further Y fall | Terrain detection | manual | Continue normal firmware hump run and observe downhill transition. | `TERRAIN = DOWNHILL` |
| S4-U03 | IMU status contains only terrain + fault bits | Buddy4 status interface | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | Motion status contains only documented motion bits | Buddy4 motion interface | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | Compact motion mapping: bit0 acceleration, bit1 turning, bit2 lift | Buddy4 motion interface | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | Current/highest hump-height getters return consistent non-negative values | Buddy4 height interface | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-M01 | Full terrain sequence | Hump / terrain detection | manual | Run robot over hump while saving serial output. Observe `FLAT -> UPHILL -> PEAK -> DOWNHILL -> FLAT`. | All terrain states occur in order and return to FLAT |
| S4-M02 | Hump-height measurement | Demo: measure hump height | manual | Measure physical hump with ruler; run robot over it; save serial output and compare reported height with reference measurement. | Hump detected and plausible calibrated height reported |
| S4-M03 | Highest-hump tracking | Mission: report highest hump | manual | Run over two or more humps where practical; save reported current/highest values. | Highest value never decreases and represents largest measured hump |
| S4-M04 | Acceleration detection | Motion classification | manual | Move/drive robot while saving serial output, then leave stationary. | Acceleration bit/event sets during motion change and clears when stationary |
| S4-M05 | Turning detection | Motion / turn-rate reporting | manual | Turn robot while saving serial output, then stop turning. | Turning bit/event sets; turn-rate reported; bit clears after turn ends |
| S4-M06 | Pickup/lift detection and safety-stop request | Updated pickup requirement | manual | After `PICKUP SAFETY ARMED`, lift robot while saving serial output. | `LIFTED` reported, motion bit2 sets, safety-stop requested and Buddy2 receives stop command |
| S4-M07 | Physical motor stop after pickup | Demo: stop motors when car is picked up | manual | With wheels physically running, record video + serial; lift robot after pickup safety is armed. | Both physical wheels stop promptly after pickup |
| S4-M08 | IMU fault reporting | IMU fault/status | manual | Reproduce an IMU read/init failure only when safe and practical; save serial output. | IMU fault bit set and fault reported |

*Method:* **auto** = `TEST_CHECK` in code, result printed on console; **manual** = a person observes the result using console output, measurement, or video. `APP_TEST` does not run the production `imu_task()` FSM; S4-IMU06–09 are therefore manual runtime tests, not automatic test-image PASS checks.

### Evidence guidance

- Save automated/test-image console output under `testing/subsystem4/logs/`.
- Save terrain, hump-height, acceleration, turning and pickup serial output under `testing/subsystem4/logs/`.
- For **S4-M07**, use a short video as primary physical evidence because console output alone cannot prove both wheels stopped. Add the unlisted video link to `videos/README.md` and reference it in the result row.
- Do not record a PASS until the test has actually been run on the stated commit. Historical results belong to their original commits, not automatically to the final cleaned `main` build.
- **21 Nov 2026:** S4-M07 passed twice based on physical observation and Buddy2 brake messages; no video supplied and stop latency was not measured. Preserve earlier failed attempts separately if their logs are available.
- **23 Nov 2026:** Diagnostic serial run on `buddy4-buddy2-integration` at `9a7ce8f` showed three completed terrain cycles and heights of 4, 2 and 6 mm. The `4385 counts = 10 mm` calibration remains provisional; a ruler comparison is still required for the full S4-M02 acceptance criterion.
- The 23 Nov serial log confirms Buddy2's brake message but does not by itself establish physical wheel-stop observation. The 21 Nov physical PASS entries remain separate.

## Results

Add a new row for every run. Never edit or delete old run records. `PARTIAL` means observed evidence does not cover every acceptance criterion; do not count it as PASS.

| Date | Test ID | Commit | Result | Log / evidence | Notes |
|---|---|---|---|---|---|
| 2026-11-21 | S4-M06 | `7c2a723` | PASS | COM9 serial, pickup test 1 | `LIFTED tilt=21 y_change=2528`; motion status `0x05`; `SAFETY STOP REQUEST result=2`; `[S2] STOP RECEIVED`. |
| 2026-11-21 | S4-M07 | `7c2a723` | PASS | COM9 serial, pickup test 1; physical observation | `[S2] BRAKE APPLIED`; Physical wheel stop was confirmed during the test. No video supplied; stop latency not measured. |
| 2026-11-21 | S4-M06 | `7c2a723` | PASS | COM9 serial, pickup test 2 | `LIFTED tilt=20 y_change=3514`; motion status `0x07`; `SAFETY STOP REQUEST result=2`; `[S2] STOP RECEIVED`. |
| 2026-11-21 | S4-M07 | `7c2a723` | PASS | COM9 serial, pickup test 2; physical observation | `[S2] BRAKE APPLIED`; Physical wheel stop was confirmed during the test. No video supplied; stop latency not measured. |
| 2026-11-23 | S4-IMU06 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | Baseline `y=-166`; initial terrain `0` (FLAT), returning to FLAT after each hump. |
| 2026-11-23 | S4-IMU07 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | Positive-Y rise produced UPHILL, e.g. baseline `-166`, filtered Y `1440`. |
| 2026-11-23 | S4-IMU08 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | UPHILL transitioned to PEAK in each of three cycles. |
| 2026-11-23 | S4-IMU09 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | PEAK transitioned to DOWNHILL in each of three cycles. |
| 2026-11-23 | S4-M01 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | Three full cycles: FLAT → UPHILL → PEAK → DOWNHILL → FLAT. |
| 2026-11-23 | S4-M02 | `9a7ce8f` | PARTIAL | `logs/2026-11-23_terrain_hump_pickup.txt` | Reported hump heights 4, 2 and 6 mm. No ruler measurement in this evidence; calibration accuracy unverified. |
| 2026-11-23 | S4-M03 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | Current heights 4 → 2 → 6 mm; highest heights 4 → 4 → 6 mm. |
| 2026-11-23 | S4-M04 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | `MOTION = ACCELERATING` and `MOTION = STATIONARY`; motion bit0 set and cleared. |
| 2026-11-23 | S4-M05 | `9a7ce8f` | PARTIAL | `logs/2026-11-23_terrain_hump_pickup.txt` | `MOTION EVENT = TURNING`, `TURN RATE = 150 deg/s`; no subsequent clear of turning bit shown. |
| 2026-11-23 | S4-M06 | `9a7ce8f` | PASS | `logs/2026-11-23_terrain_hump_pickup.txt` | After `PICKUP SAFETY ARMED`: `LIFTED tilt=38 y_change=8557`, motion status `0x07`, safety-stop request result `2`, `[S2] STOP RECEIVED`. |
| 2026-11-23 | S4-M07 | `9a7ce8f` | PARTIAL | `logs/2026-11-23_terrain_hump_pickup.txt` | `[S2] BRAKE APPLIED` recorded; this serial log alone does not verify physical wheel stop or latency. |
| 2026-11-24 | S4-M01 | `55ea766` | PASS | `logs/2026-11-24_terrain_pickup.txt`; https://youtu.be/QnVUMnxhetU | Full terrain sequence FLAT → UPHILL → PEAK → DOWNHILL → FLAT; estimated hump height 2 mm. |
| 2026-11-24 | S4-M06 | `55ea766` | PASS | `logs/2026-11-24_terrain_pickup.txt`; https://youtu.be/QnVUMnxhetU | Pickup detected at tilt=71°, y_change=13216, motion status=0x07; safety-stop result=2; Buddy2 STOP RECEIVED and BRAKE APPLIED. |
