# Subsystem 2 — test log

**Subsystem:** Producer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | stop() is accepted and completes within 200 ms | IF-06 | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | idle with zero encoder errors | encoder integration | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | 100 mm move finishes, both encoders count up, L/R within 10% | distance estimation | auto | subsystem2_test.c, wheels in the air | `[TEST] S2-U05a/b/c PASS` |
| S2-U06 | wait_done returns the error of a failed command | IF-06 | auto | subsystem2_test.c | `[TEST] S2-U06 PASS` | 
| S2-U07 | both motors spin forward for a positive duty | motor driver integration | manual | `S2_BRINGUP 1`, `S2_BRINGUP_DUTY 350`, wheels in the air. Watch each wheel | both wheels turn forward, 2 sec on and 2 sec off |
| S2-U08 | both encoders count while the motors run | encoder integration | manual | same build as U07. Read `count L=` / `count R=` during `duty=350` | both counts rise at a similar rate, stop when `duty=0`,`err=0` |
| S2-U09 | encoder counts per wheel turn | motion calibration | manual | `S2_BRINGUP 1`, `S2_BRINGUP_DUTY 0`. Used post it note to mark the tyre, turn each wheel forward 10 turns by hand, read `count L=` / `count R=` | left and right within 5% |
| S2-U10 | wheel diameter | motion calibration | manual | measure across the tyre with a ruler | get cm and convert value to mm |
| S2-U11 | track width | motion calibration | manual | measure from the centre of the left tyre to the centre of the right tyre in cm and convert to mm | track width recorded in mm | 



*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-07 | S2-U07 | f1437b1 | Zhen Hui | PASS | serial monitor | Both wheels turn forward. MOTOR_RIGHT_INVERT = 1. |
| 2026-11-07 | S2-U08 | f1437b1 | Zhen Hui | FAIL | serial monitor | Right encoder (Motor B) gave no signal: `count R` stayed 0. Tried Grove 6, Grove 2 and a new Grove cable, Motor A worked in the same places. Also fixed pull down bug in `drv_encoder.c`. Motor B to be replaced |
| 2026-11-10 | S2-U08 | f1437b1 | Zhen Hui | PASS | `logs/2026-11-10_S2-U08_encoders.txt` | New Motor B fitted. L ≈ 361/line, R ≈ 370/line at duty 350; err = 0. B-only mode (Motor A phase A gives no signal) |
| 2026-11-10 | S2-U09 | f1437b1 | Zhen Hui | PASS | `logs/2026-11-10_S2-U09_counts_per_turn.txt` | Left 0→12858 = 1286/turn; Right 4→12728 = 1272/turn; difference 1.1%. Using average 1279 |
| 2026-11-10 | S2-U10 | f1437b1 | Zhen Hui | PASS | ruler measurement | 5.4 cm = 54 mm |
| 2026-11-10 | S2-U11 | f1437b1 | Zhen Hui | PASS | ruler measurement | 9.6 cm = 96 mm |
