# Subsystem 2 — test log

**Subsystem:** Producer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | stop() is accepted and completes within 200 ms | IF-06 | auto | subsystem2_test.c | [TEST] S2-U03 PASS |
| S2-U04 | idle with zero encoder errors | encoder integration | auto | subsystem2_test.c | [TEST] S2-U04 PASS |
| S2-U05 | 100 mm move finishes, both encoders count up, L/R within 10% | distance estimation | auto | subsystem2_test.c, wheels in the air | [TEST] S2-U05a/b/c PASS |
| S2-U06 | wait_done returns the error of a failed command | IF-06 | auto | subsystem2_test.c | [TEST] S2-U06 PASS | 
| S2-U07 | both motors spin forward for a positive duty | motor driver integration | manual | in subsytem2.h, S2_BRINGUP 1 and S2_BRINGUP_DUTY 350, wheels in the air | both wheels move forward for 2 sec then stop and continue again |
| S2-U08 | both encoders count while the motors run | encoder integration | manual | same build as S2-U07. Read count L= and count R= during duty=350 | both counts rise at a similar rate, stop when duty=0, err=0 | 
| S2-U09 | encoder counts per wheel turn | motion calibration | manual | in subsystem2.h, S2_BRINGUP 1, S2_BRINGUP_DUTY 0. Used post it note to mark the tyre, turn each wheel forward 10 turns by hand, read count L= and count R= | left and right within 5% |
| S2-U10 | wheel diameter | motion calibration | manual | measure across the tyre with a ruler | diameter value in cm and convert to mm |
| S2-U11 | track width | motion calibration | manual | measure from the centre of the left tyre to the centre of the right tyre with a ruler| track width value in cm and convert to mm | 



*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-07 | S2-U07 | f1437b1 | Zhen Hui | PASS | | both wheels turn forward. MOTOR_RIGHT_INVERT = 1. |
| 2026-11-07 | S2-U08 | f1437b1 | Zhen Hui | FAIL | logs/2026-11-07_S2-U08_right_encoders_fail.txt | motor A worked but the right encoder for motor b have no signal as count R remained at 0. Tried to troubleshoot by using another grove port and a new grove cable. Motor b might need a replacement. Also fixed pull down bug in drv_encoder.c |
| 2026-11-10 | S2-U08 | f1437b1 | Zhen Hui | PASS | logs/2026-11-10_S2-U08_both_encoders_pass.jpeg | replaced motor b. L ≈ 361/line, R ≈ 370/line at duty 350; err = 0. B-only mode,motor a phase a gives no signal |
| 2026-11-10 | S2-U09 | f1437b1 | Zhen Hui | PASS | logs/2026-11-10_S2-U09_counts_per_turn.jpeg | left increases from 0 to 12858. 12858/10 = 1286 counts per turn, right increases from 0 to 12724. 12724/10 = 1272 counts per turn, using average = 1279 | 
| 2026-11-10 | S2-U10 | f1437b1 | Zhen Hui | PASS | ruler measurement | 5.4 cm = 54 mm |
| 2026-11-10 | S2-U11 | f1437b1 | Zhen Hui | PASS | ruler measurement | 9.6 cm = 96 mm | 
